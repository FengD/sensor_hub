#include <vector>
#include "lidar_drivers/parser/falcon/falcon.h"
#include "lidar_drivers/parser/falcon/nps_adjustment.h"
#include "lidar_drivers/proto/lidar_config.pb.h"

namespace crdc {
namespace airi {

LidarParserFalcon::LidarParserFalcon() {
  lidar_point_num_ = 0;
  last_frame_id_ = 0;
  max_range_ = 2000.0;
  min_range_ = 0.4;
  // used for init the nps adjustment
  init_f_falcon();
  // used for init the sin cos table
  init_sin_coss_table();
  xyz_from_sphere_.resize(BUFFER_SIZE);
}

LidarParserFalcon::~LidarParserFalcon() {
  xyz_from_sphere_.clear();
}

void LidarParserFalcon::init_f_falcon() {
  for (uint32_t ich = 0; ich < INNO_CHANNEL_NUMBER; ++ich) {
    v_angle_offset_[INNO_ITEM_TYPE_SPHERE_POINTCLOUD][ich] = ich * VAngleDiffBase;
  }
  // init the nps adjustment
  memset(nps_adjustment_, 0, sizeof(nps_adjustment_));
  for (uint32_t v = 0; v < VTableEffeHalfSize * 2 + 1; ++v) {
    for (uint32_t h = 0; h < HTableEffeHalfSize * 2 + 1; ++h) {
      for (uint32_t ich = 0; ich < INNO_CHANNEL_NUMBER; ++ich) {
        for (uint32_t xz = 0; xz < XZSize; ++xz) {
          double k = kInnoPs2Nps[xz][ich][v][h];
          double u = k / AdjustUnitInMeter;
          double q = std::floor(u + 0.5);
          nps_adjustment_[v][h][ich][xz] = q;
        }
      }
    }
  }
}

void LidarParserFalcon::init_sin_coss_table() {
  for (int32_t i = 0; i <= AngleTableSize; ++i) {
    double angle = i * RadPerInnoAngleUnit;
    cos_table_[i] = cos(angle);
    sin_table_[i] = sin(angle);
  }
}

bool LidarParserFalcon::init_lidar_parser() {
  return true;
}

bool LidarParserFalcon::init_calib() {
  return true;
}

void LidarParserFalcon::get_block_info(const uint8_t* data, LidarParserInfo& parser_info) {
  (void)data;
  (void)parser_info;
}

void LidarParserFalcon::get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) {
  (void)data;
  (void)parser_info;
}

void LidarParserFalcon::calibrate_point(LidarParserInfo& parser_info) {
  (void)parser_info;
}

uint64_t LidarParserFalcon::get_packet_timestamp(const Packet* packet) {
#ifdef WITH_ROS2
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data.data());
#else
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data().data());
#endif
  const InnoDataPacket* inno_data_packet = reinterpret_cast<const InnoDataPacket*>(data);
  // micro second
  uint64_t ts_us = inno_data_packet->common.ts_start_us;
  return ts_us;
}

bool LidarParserFalcon::init_pool() {
  cloud_pool_.reset(new common::CCObjectPool<LidarPointCloud>(config_.pool_size()));
  cloud_pool_->ConstructAll();
  std::vector<std::shared_ptr<LidarPointCloud>> temp_clouds;
#ifdef WITH_ROS2
  sensor_msgs::msg::PointField f;
#endif
  for (auto i = 0; i < config_.pool_size(); ++i) {
    auto cloud = cloud_pool_->GetObject();
    if (cloud == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id() << "] failed to getobject: " << i;
      return false;
    }

#ifdef WITH_ROS2
    cloud->cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->cloud_msg_->fields.clear();
    add_field(f, "sec", sensor_msgs::msg::PointField::UINT32, 0, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "usec", sensor_msgs::msg::PointField::UINT32, 4, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "x", sensor_msgs::msg::PointField::FLOAT32, 8, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "y", sensor_msgs::msg::PointField::FLOAT32, 12, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "z", sensor_msgs::msg::PointField::FLOAT32, 16, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "intensity", sensor_msgs::msg::PointField::FLOAT32, 20, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "scan_id", sensor_msgs::msg::PointField::FLOAT32, 24, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "scan_idx", sensor_msgs::msg::PointField::FLOAT32, 28, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "flags", sensor_msgs::msg::PointField::FLOAT32, 32, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "is_2nd_return", sensor_msgs::msg::PointField::FLOAT32, 36, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "elongation", sensor_msgs::msg::PointField::FLOAT32, 40, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    cloud->cloud_msg_->data.resize(config_.max_points() * sizeof(SolidStateLidarPoint));
    cloud->cloud_msg_->point_step = sizeof(SolidStateLidarPoint);
    cloud->cloud_msg_->height =  config_.lidar_packet_config().lasers();
    cloud->cloud_msg_->is_dense = true;
    cloud->cloud_msg_->header.frame_id = config_.frame_id();
    cloud->cloud_msg_->is_bigendian = false;
#else
    cloud->proto_cloud_ = std::make_shared<PointCloud2>();
    cloud->proto_cloud_->clear_fields();
    PointField* f;
    ADD_FIELD(cloud->proto_cloud_, "sec", PointField::UINT32, 0, 1);
    ADD_FIELD(cloud->proto_cloud_, "usec", PointField::UINT32, 4, 1);
    ADD_FIELD(cloud->proto_cloud_, "x", PointField::FLOAT32, 8, 1);
    ADD_FIELD(cloud->proto_cloud_, "y", PointField::FLOAT32, 12, 1);
    ADD_FIELD(cloud->proto_cloud_, "z", PointField::FLOAT32, 16, 1);
    ADD_FIELD(cloud->proto_cloud_, "intensity", PointField::FLOAT32, 20, 1);
    ADD_FIELD(cloud->proto_cloud_, "scan_id", PointField::FLOAT32, 24, 1);
    ADD_FIELD(cloud->proto_cloud_, "scan_idx", PointField::FLOAT32, 28, 1);
    ADD_FIELD(cloud->proto_cloud_, "flags", PointField::FLOAT32, 32, 1);
    ADD_FIELD(cloud->proto_cloud_, "is_2nd_return", PointField::FLOAT32, 36, 1);
    ADD_FIELD(cloud->proto_cloud_, "elongation", PointField::FLOAT32, 40, 1);
    cloud->proto_cloud_->mutable_data()->resize(config_.max_points() * sizeof(LidarPoint));
    cloud->proto_cloud_->set_point_step(sizeof(LidarPoint));
    cloud->proto_cloud_->set_height(config_.lidar_packet_config().lasers());
    cloud->proto_cloud_->set_is_dense(true);
    cloud->proto_cloud_->mutable_header()->set_frame_id(config_.frame_id());
    cloud->proto_cloud_->set_is_bigendian(false);
#endif
    temp_clouds.emplace_back(cloud);
  }
  temp_clouds.clear();
  return true;
}

uint32_t LidarParserFalcon::crc32_do(uint32_t crc, const void *const buf,
                             const size_t buf_len) {
  const uint8_t *data = reinterpret_cast<const uint8_t*>(buf);
  size_t bytes = buf_len;
  // Calculate the crc for the data
  while (bytes--) {
    crc = crctable[(crc ^ (*data++)) & 0xff] ^ (crc >> 8);
  }
  return crc;
}

bool LidarParserFalcon::is_lidar_packet_valid(const Packet* packet) {
#ifdef WITH_ROS2
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data.data());
#else
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data().data());
#endif
  const InnoDataPacket* inno_data_packet = reinterpret_cast<const InnoDataPacket*>(data);
  // now only support parser lidar data
  int32_t type = inno_data_packet->type;
  if (type != config_.lidar_packet_config().data_type()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] data_type " << type
               << " wrong " << config_.lidar_packet_config().data_type();
    return false;
  }

  uint16_t magic_number = inno_data_packet->common.version.magic_number;
  if (inno_data_packet->common.version.magic_number != config_.lidar_packet_config().version()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] magic number " << magic_number
               << " wrong " << config_.lidar_packet_config().version();
    return false;
  }

  uint16_t block_size = inno_data_packet->item_size;
  if (block_size != config_.lidar_packet_config().block_size()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] block_size " << block_size
               << " wrong " << config_.lidar_packet_config().block_size();
    return false;
  }

  uint32_t size = inno_data_packet->common.size;
  if (size != inno_data_packet->item_number * config_.lidar_packet_config().block_size() +
              sizeof(InnoDataPacket)) {
    LOG(ERROR) << "[" << config_.frame_id() << "] size " << size
               << " wrong";
    return false;
  }

  // calculate crc32
  uint32_t crc = 0xffffffffu;
  crc = crc32_do(crc, &(inno_data_packet->common.version),
                sizeof(inno_data_packet->common.version));
  size_t off = offsetof(struct InnoCommonHeader, size);
  crc = crc32_do(crc, &(inno_data_packet->common.size),
                inno_data_packet->common.size - off);
  uint32_t checksum = crc ^ 0xffffffffu;
  uint32_t header_checksum = (uint32_t)inno_data_packet->common.checksum;
  if (header_checksum != checksum) {
    LOG(ERROR) << "[" << config_.frame_id() << "] checksum " << header_checksum
               << " wrong " << checksum;
    return false;
  }
  return true;
}

bool LidarParserFalcon::set_point_cloud(std::shared_ptr<LidarPointCloud>* &cloud, bool &ret,
                                          double height, const Packet* packet) {
#ifdef WITH_ROS2
  cloud_->cloud_msg_->header.stamp.sec = frame_start_utime_ / 1000000;
  cloud_->cloud_msg_->header.stamp.nanosec = frame_start_utime_ % 1000000 * 1000;
  cloud_->cloud_msg_->data.resize(lidar_point_num_ * sizeof(SolidStateLidarPoint));
  cloud_->cloud_msg_->width = lidar_point_num_;
  cloud_->cloud_msg_->height = height;
  cloud_->cloud_msg_->row_step = sizeof(SolidStateLidarPoint) * cloud_->cloud_msg_->width;
  cloud_->last_packet_utime_ = packet->time_system;
  *cloud = cloud_;
  ret = true;
  if (!update_frame(packet->time_system)) {
    return false;
  }
#endif
  return true;
}

SolidStateLidarPoint LidarParserFalcon::set_lidar_point(const InnoXyzPoint* pt) {
  SolidStateLidarPoint lidar_point;
  lidar_point.x_ = pt->y;  // right
  lidar_point.y_ = pt->z;  // forward
  lidar_point.z_ = pt->x;  // up
  lidar_point.intensity_ = static_cast<float>(pt->refl);
  int32_t roi = pt->in_roi == 3 ? (1 << 2) : 0;
  lidar_point.scan_id_ = static_cast<float>(pt->scan_id);
  lidar_point.scan_idx_ = static_cast<float>(pt->scan_idx);
  lidar_point.flags_ = static_cast<float>(pt->channel | roi | (pt->facet << 3) | (pt->type << 6));
  lidar_point.is_2nd_return_ = static_cast<float>(pt->is_2nd_return);
  lidar_point.elongation_ = static_cast<float>(pt->elongation);
  lidar_point.timestamp_ = frame_start_utime_;
  return lidar_point;
}

bool LidarParserFalcon::is_frame_end(const InnoDataPacket* pkt) {
  if (last_frame_id_ == 0) {
    last_frame_id_ = pkt->idx;
  }
  if (last_frame_id_ != pkt->idx) {
    last_frame_id_ = pkt->idx;
    return true;
  }
  return false;
}

void LidarParserFalcon::lookup_xz_adjustment(const InnoBlockAngles &angles, uint32_t ch,
                            double *x, double *z) {
  uint32_t v = angles.v_angle / 512;
  uint32_t h = angles.h_angle / 512;
  v += VTableEffeHalfSize;
  h += HTableEffeHalfSize;
  // avoid index out-of-bound
  v = v & (VTableSize - 1);
  h = h & (HTableSize - 1);
  int8_t *addr_x = &nps_adjustment_[v][h][ch][0];
  int8_t *addr_z = addr_x + 1;
  *x = *addr_x * AdjustUnitInMeter;
  *z = *addr_z * AdjustUnitInMeter;
  return;
}

void LidarParserFalcon::calculate_points_data_xyz(const InnoBlockAngles &angles,
                        const uint32_t radius_unit, const uint32_t channel, InnoXyzrD *result) {
  result->radius = radius_unit * MeterPerInnoDistanceUnit;
  double t;
  if (angles.v_angle >= 0) {
    t = result->radius * cos_table_[angles.v_angle];
    result->x = result->radius * sin_table_[angles.v_angle];
  } else {
    t = result->radius * cos_table_[-angles.v_angle];
    result->x = -result->radius * sin_table_[-angles.v_angle];
  }
  if (angles.h_angle >= 0) {
    result->y = t * sin_table_[angles.h_angle];
    result->z = t * cos_table_[angles.h_angle];
  } else {
    result->y = -t * sin_table_[-angles.h_angle];
    result->z = t * cos_table_[-angles.h_angle];
  }
  double x_adj, z_adj;
  lookup_xz_adjustment(angles, channel, &x_adj, &z_adj);
  result->x += x_adj;
  result->z += z_adj;
  return;
}

// convert sphere to xyz
bool LidarParserFalcon::convert_to_xyz_pointcloud(const InnoDataPacket &src,
                                InnoDataPacket *dest, size_t dest_size) {
  size_t required_size = sizeof(InnoDataPacket);
  if (required_size > dest_size) {
    LOG(ERROR) << "Not enough size, required: " << required_size;
    return false;
  }
  // copy common and innodatapacket messgae
  memcpy(dest, &src, sizeof(InnoDataPacket));
  if (src.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
    dest->type = INNO_ITEM_TYPE_XYZ_POINTCLOUD;
    dest->item_size = sizeof(InnoXyzPoint);
  }
  dest->item_number = 0;
  if (src.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
    uint32_t unit_size = 0;
    uint32_t mode = InnoMultipleReturnMode(src.multi_return_mode);
    uint32_t mr = mode >= INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ? 2 : 1;
    if (mr == 1) {
      unit_size = config_.lidar_packet_config().block_size();
    } else {
      // multi return mode
      LOG(ERROR) << "return mode is wrong";
    }
    const InnoBlock* block = reinterpret_cast<const InnoBlock*>(reinterpret_cast<const char*>(&src)
                                                                        + sizeof(InnoDataPacket));
    for (size_t i = 0; i < src.item_number; ++i, block = reinterpret_cast<const InnoBlock*>
                                        (reinterpret_cast<const char*>(block) + unit_size)) {
      InnoBlockFullAngles full_angles;
      get_block_full_angles(&full_angles, block->header);
      uint16_t time_adjust_10us = block->header.ts_10us;
      for (uint32_t ch = 0; ch < INNO_CHANNEL_NUMBER; ch++) {
        for (uint32_t m = 0; m < mr; m++) {
          const InnoChannelPoint &pt = block->points[innoblock_get_idx(ch, m)];
          if (pt.radius > 0) {
            InnoXyzPoint* xyz_points = reinterpret_cast<InnoXyzPoint*>
                                      (reinterpret_cast<char*>(dest) + sizeof(InnoDataPacket));
            InnoXyzPoint &ipt = xyz_points[dest->item_number];
            required_size += sizeof(InnoXyzPoint);
            if (required_size > dest_size) {
              LOG(ERROR) << "not enough size, required: " << required_size;
              return false;
            }
            get_xyz_point(block->header, pt, full_angles.angles[ch], ch, &ipt);
            ipt.multi_return = m;
            ipt.is_2nd_return = pt.is_2nd_return;
            ipt.ts_10us = time_adjust_10us;
            dest->item_number++;
          }
        }
      }
    }
  }
  dest->common.size = required_size;
  return true;
}

bool LidarParserFalcon::parse_lidar_packet(const Packet* packet,
                        std::shared_ptr<LidarPointCloud>* cloud) {
  bool ret = false;
  if (!is_lidar_packet_valid(packet)) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet is invalid.";
    return ret;
  }

  if (lidar_point_num_ > static_cast<int32_t>(config_.max_points())) {
    LOG(ERROR) << "[" << config_.frame_id() << "] lidar point is close to max ["
               << lidar_point_num_ << ", " << config_.max_points() << "]";
  }

  auto &packet_config = config_.lidar_packet_config();
#ifdef WITH_ROS2
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data.data());
#else
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data().data());
#endif
  const InnoDataPacket* inno_data_packet = reinterpret_cast<const InnoDataPacket*>(data);
  frame_start_utime_ = get_packet_timestamp(packet);
  // convert sphere point to xyz point
  convert_to_xyz_pointcloud(*inno_data_packet,
            reinterpret_cast<InnoDataPacket*>(&xyz_from_sphere_[0]), xyz_from_sphere_.size());
  InnoDataPacket* output_data_packet = reinterpret_cast<InnoDataPacket*>(&xyz_from_sphere_[0]);
  if (unlikely(cloud_ == nullptr)) {
    if (!is_frame_end(output_data_packet)) {
      return false;
    }

    if (!update_frame(frame_start_utime_)) {
      return false;
    }
  } else {
    if (is_frame_end(output_data_packet)) {
      // use system time if sensor time is invalid
      if (config_.use_local_time()) {
        auto now = get_now_microsecond();
        if (fabs(now - frame_start_utime_) > 10000000) frame_start_utime_ = now;
      }
      if (!set_point_cloud(cloud, ret, packet_config.lasers(), packet)) return false;
    }
  }
  const InnoXyzPoint* pt = reinterpret_cast<const InnoXyzPoint*>
                    (reinterpret_cast<const char*>(output_data_packet) + sizeof(InnoDataPacket));
  for (uint32_t i = 0; i < output_data_packet->item_number; ++i, ++pt) {
    if (is_invalid_point(pt)) {
      continue;
    }
    SolidStateLidarPoint &lidar_point = solid_lidar_point_[lidar_point_num_];
    lidar_point = set_lidar_point(pt);
    lidar_point_num_++;
  }
  return ret;
}

}  // namespace airi
}  // namespace crdc
