#pragma once
#include <memory>
#include <string>
#include <vector>
#include "common/common.h"
#include "lidar_drivers/parser/parser.h"

namespace crdc {
namespace airi {

#define INNO_CHANNEL_NUMBER_BIT 2
#define INNO_CHANNEL_NUMBER (1 << INNO_CHANNEL_NUMBER_BIT)
#define M_PI 3.14159265358979323846
#define DistanceUnitPerMeter 200
#define AngleUnitPerPiRad 32768
#define AdjustUnitInMeter 0.0025
#define VAngleDiffBase 196
#define XZSize 2
#define VTableEffeHalfSize 6
#define HTableEffeHalfSize 22
#define VTableSizeBits 4
#define HTableSizeBits 6
constexpr double RadPerInnoAngleUnit = M_PI / AngleUnitPerPiRad;
constexpr int32_t AngleTableSize = 2 * AngleUnitPerPiRad;
constexpr double MeterPerInnoDistanceUnit = 1.0 / DistanceUnitPerMeter;
constexpr uint32_t VTableSize = 1 << VTableSizeBits;
constexpr uint32_t HTableSize = 1 << HTableSizeBits;

enum InnoItemType {
  INNO_ITEM_TYPE_NONE = 0,
  INNO_ITEM_TYPE_SPHERE_POINTCLOUD = 1,
  INNO_ITEM_TYPE_MESSAGE = 2,
  INNO_ITEM_TYPE_MESSAGE_LOG = 3,
  INNO_ITEM_TYPE_XYZ_POINTCLOUD = 4,
  INNO_ITEM_TYPE_MAX = 13,
};

enum InnoMultipleReturnMode {
  INNO_MULTIPLE_RETURN_MODE_NONE = 0,
  INNO_MULTIPLE_RETURN_MODE_SINGLE = 1,
  INNO_MULTIPLE_RETURN_MODE_2_STRONGEST = 2,
  INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST = 3,
  INNO_MULTIPLE_RETURN_MODE_MAX,
};

struct InnoBlockAngles {
  int16_t h_angle;
  int16_t v_angle;
  InnoBlockAngles() {}
};

struct InnoBlockFullAngles {
  InnoBlockAngles angles[INNO_CHANNEL_NUMBER];
  InnoBlockFullAngles() {}
};

struct InnoCommonVersion {
  uint16_t magic_number;
  uint8_t major_version;
  uint8_t minor_version;
  uint16_t fw_sequence;
  InnoCommonVersion() {}
}__attribute__((packed));

struct InnoCommonHeader {
  InnoCommonVersion version;
  uint32_t checksum;
  uint32_t size;
  uint8_t source_id : 4;
  uint8_t timestamp_sync_type : 4;
  uint8_t reserved;
  double ts_start_us;
  uint8_t lidar_mode;
  uint8_t lidar_status;
  InnoCommonHeader() {}
}__attribute__((packed));

struct InnoDataPacket {
  InnoCommonHeader common;
  uint64_t idx;
  uint16_t sub_idx;
  uint16_t sub_seq;
  uint32_t type : 8;
  uint32_t item_number : 24;
  uint16_t item_size;
  uint32_t topic;
  uint16_t scanner_direction : 1;
  uint16_t use_reflectance   : 1;
  uint16_t multi_return_mode : 3;
  uint16_t confidence_level  : 2;
  uint16_t is_last_sub_frame : 1;
  uint16_t is_last_sequence  : 1;
  uint16_t has_tail : 1;
  uint16_t frame_sync_locked : 1;
  uint16_t is_first_sub_frame : 1;
  uint16_t last_four_channel : 1;
  uint16_t reserved_flag : 3;
  int16_t roi_h_angle;
  int16_t roi_v_angle;
  InnoDataPacket() {}
}__attribute__((packed));

struct InnoBlockHeader {
  int16_t h_angle;
  int16_t v_angle;
  uint16_t ts_10us;
  uint16_t scan_idx;
  uint16_t scan_id: 9;
  int64_t h_angle_diff_1 : 9;
  int64_t h_angle_diff_2 : 10;
  int64_t h_angle_diff_3 : 11;
  int64_t v_angle_diff_1 : 8;
  int64_t v_angle_diff_2 : 9;
  int64_t v_angle_diff_3 : 9;
  uint64_t in_roi : 2;
  uint64_t facet : 3;
  uint64_t reserved_flags : 2;
  InnoBlockHeader() {}
}__attribute__((packed));

struct InnoChannelPoint {
  uint32_t radius : 17;
  uint32_t refl : 8;
  uint32_t is_2nd_return : 1;
  uint32_t type : 2;
  uint32_t elongation : 4;
  InnoChannelPoint() {}
}__attribute__((packed));

struct InnoBlock {
  InnoBlockHeader header;
  InnoChannelPoint points[INNO_CHANNEL_NUMBER];
}__attribute__((packed));

struct InnoXyzPoint {
  float x;
  float y;
  float z;
  float radius;
  uint16_t ts_10us;
  uint16_t scan_id : 9;
  uint16_t in_roi : 2;
  uint16_t facet : 3;
  uint16_t multi_return : 1;
  uint16_t reserved_flags : 1;
  uint32_t is_2nd_return : 1;
  uint32_t scan_idx : 14;
  uint32_t refl : 9;
  uint32_t type : 2;
  uint32_t elongation : 4;
  uint32_t channel : 2;
}__attribute__((packed));

struct InnoXyzrD {
  double x;
  double y;
  double z;
  double radius;
}__attribute__((packed));

class LidarParserFalcon : public LidarParser {
 public:
  LidarParserFalcon();
  virtual ~LidarParserFalcon();

  bool is_lidar_packet_valid(const Packet* packet) override;

  bool parse_lidar_packet(const Packet* packet,
                                std::shared_ptr<LidarPointCloud>* cloud) override;

  bool init_lidar_parser() override;

  bool init_calib() override;

  bool init_pool() override;

  void get_block_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  void get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  uint64_t get_packet_timestamp(const Packet* packet) override;

  void calibrate_point(LidarParserInfo& parser_info) override;

  bool is_frame_end(const InnoDataPacket* pkt);

  std::string get_name() const override {
    return "LidarParserFalcon";
  }

 protected:
  inline bool update_frame(const uint64_t& packet_time) {
    if (!frame_init()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to init frame.";
      return false;
    }
    cloud_->first_packet_utime_ = packet_time;
    return true;
  }

  inline bool frame_init() {
    cloud_ = cloud_pool_->GetObject();
    if (cloud_ == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to get cloud data.";
      return false;
    }
#ifdef WITH_ROS2
    const SolidStateLidarPoint* temp = reinterpret_cast<const SolidStateLidarPoint*>
                                                       (cloud_->cloud_msg_->data.data());
    solid_lidar_point_ = const_cast<SolidStateLidarPoint*>(temp);
    cloud_->cloud_msg_->data.resize(config_.max_points() * sizeof(SolidStateLidarPoint));
#else
    const SolidStateLidarPoint* temp = reinterpret_cast<const SolidStateLidarPoint*>(
                                              cloud_->proto_cloud_->mutable_data()->data());
    solid_lidar_point_ = const_cast<SolidStateLidarPoint*>(temp);
    cloud_->proto_cloud_->mutable_data()->resize(config_.max_points() *
                                                sizeof(SolidStateLidarPoint));
#endif
    lidar_point_num_ = 0;
    frame_valid_pts_ = valid_pts_;
    frame_invalid_pts_ = invalid_pts_;
    invalid_pts_ = 0;
    valid_pts_ = 0;
    return true;
  }

bool is_invalid_point(const InnoXyzPoint* pt) {
  if (pt->radius > max_range_ || pt->radius < min_range_) {
    invalid_pts_++;
    return true;
  }
  if (pt->channel >= INNO_CHANNEL_NUMBER) {
    invalid_pts_++;
    return true;
  }
  valid_pts_++;
  return false;
}

  void calculate_points_data_xyz(const InnoBlockAngles &angles, const uint32_t radius_unit,
                      const uint32_t channel, InnoXyzrD *result);

 private:
  inline void get_block_full_angles(InnoBlockFullAngles *full, const InnoBlockHeader &b) {
    full->angles[0].h_angle = b.h_angle;
    full->angles[0].v_angle = b.v_angle;
    full->angles[1].h_angle = b.h_angle + b.h_angle_diff_1;
    full->angles[1].v_angle = b.v_angle + b.v_angle_diff_1 + v_angle_offset_[1][1];
    full->angles[2].h_angle = b.h_angle + b.h_angle_diff_2;
    full->angles[2].v_angle = b.v_angle + b.v_angle_diff_2 + v_angle_offset_[1][2];
    full->angles[3].h_angle = b.h_angle + b.h_angle_diff_3;
    full->angles[3].v_angle = b.v_angle + b.v_angle_diff_3 + v_angle_offset_[1][3];
  }

  inline size_t innoblock_get_idx(size_t channel, size_t r) {
    return channel + (r << INNO_CHANNEL_NUMBER_BIT);
  }

  inline void get_xyz_point(const InnoBlockHeader &block, const InnoChannelPoint &cp,
                          const InnoBlockAngles angles, const uint32_t channel, InnoXyzPoint *pt) {
    InnoXyzrD xyzr;
    calculate_points_data_xyz(angles, cp.radius, channel, &xyzr);
    pt->x = xyzr.x;
    pt->y = xyzr.y;
    pt->z = xyzr.z;
    pt->radius = xyzr.radius;
    pt->ts_10us = block.ts_10us;
    pt->scan_idx = block.scan_idx;
    pt->scan_id = block.scan_id;
    pt->in_roi = block.in_roi;
    pt->facet = block.facet;
    pt->reserved_flags = block.reserved_flags;
    pt->refl = cp.refl;
    pt->type = cp.type;
    pt->elongation = cp.elongation;
    pt->channel = channel;
    pt->is_2nd_return = cp.is_2nd_return;
  }

  void init_f_falcon();
  void init_sin_coss_table();

  void lookup_xz_adjustment(const InnoBlockAngles &angles, uint32_t ch,
                            double *x, double *z);

  bool set_point_cloud(std::shared_ptr<LidarPointCloud>* &cloud, bool &ret,
                                          double height, const Packet* packet);

  SolidStateLidarPoint set_lidar_point(const InnoXyzPoint* pt);

  bool convert_to_xyz_pointcloud(const InnoDataPacket &src,
                                InnoDataPacket *dest, size_t dest_size);

  uint32_t crc32_do(uint32_t crc, const void *const buf, const size_t buf_len);

  int32_t lidar_point_num_;
  uint64_t last_frame_id_;
  float max_range_;
  float min_range_;
  int8_t nps_adjustment_[VTableSize][HTableSize][INNO_CHANNEL_NUMBER][XZSize];
  int32_t v_angle_offset_[INNO_ITEM_TYPE_MAX][INNO_CHANNEL_NUMBER];
  float sin_table_[AngleTableSize +1 ];
  float cos_table_[AngleTableSize + 1];
  SolidStateLidarPoint* solid_lidar_point_;
  std::vector<uint8_t> xyz_from_sphere_;
  static constexpr uint64_t BUFFER_SIZE = 1024 * 1024 * 10;
  // Table of g_crc32_1EDC6F41. Used for crc checksum
  const uint64_t crctable[256] = {
    0x00000000, 0xf26b8303, 0xe13b70f7, 0x1350f3f4, 0xc79a971f, 0x35f1141c,
    0x26a1e7e8, 0xd4ca64eb, 0x8ad958cf, 0x78b2dbcc, 0x6be22838, 0x9989ab3b,
    0x4d43cfd0, 0xbf284cd3, 0xac78bf27, 0x5e133c24, 0x105ec76f, 0xe235446c,
    0xf165b798, 0x030e349b, 0xd7c45070, 0x25afd373, 0x36ff2087, 0xc494a384,
    0x9a879fa0, 0x68ec1ca3, 0x7bbcef57, 0x89d76c54, 0x5d1d08bf, 0xaf768bbc,
    0xbc267848, 0x4e4dfb4b, 0x20bd8ede, 0xd2d60ddd, 0xc186fe29, 0x33ed7d2a,
    0xe72719c1, 0x154c9ac2, 0x061c6936, 0xf477ea35, 0xaa64d611, 0x580f5512,
    0x4b5fa6e6, 0xb93425e5, 0x6dfe410e, 0x9f95c20d, 0x8cc531f9, 0x7eaeb2fa,
    0x30e349b1, 0xc288cab2, 0xd1d83946, 0x23b3ba45, 0xf779deae, 0x05125dad,
    0x1642ae59, 0xe4292d5a, 0xba3a117e, 0x4851927d, 0x5b016189, 0xa96ae28a,
    0x7da08661, 0x8fcb0562, 0x9c9bf696, 0x6ef07595, 0x417b1dbc, 0xb3109ebf,
    0xa0406d4b, 0x522bee48, 0x86e18aa3, 0x748a09a0, 0x67dafa54, 0x95b17957,
    0xcba24573, 0x39c9c670, 0x2a993584, 0xd8f2b687, 0x0c38d26c, 0xfe53516f,
    0xed03a29b, 0x1f682198, 0x5125dad3, 0xa34e59d0, 0xb01eaa24, 0x42752927,
    0x96bf4dcc, 0x64d4cecf, 0x77843d3b, 0x85efbe38, 0xdbfc821c, 0x2997011f,
    0x3ac7f2eb, 0xc8ac71e8, 0x1c661503, 0xee0d9600, 0xfd5d65f4, 0x0f36e6f7,
    0x61c69362, 0x93ad1061, 0x80fde395, 0x72966096, 0xa65c047d, 0x5437877e,
    0x4767748a, 0xb50cf789, 0xeb1fcbad, 0x197448ae, 0x0a24bb5a, 0xf84f3859,
    0x2c855cb2, 0xdeeedfb1, 0xcdbe2c45, 0x3fd5af46, 0x7198540d, 0x83f3d70e,
    0x90a324fa, 0x62c8a7f9, 0xb602c312, 0x44694011, 0x5739b3e5, 0xa55230e6,
    0xfb410cc2, 0x092a8fc1, 0x1a7a7c35, 0xe811ff36, 0x3cdb9bdd, 0xceb018de,
    0xdde0eb2a, 0x2f8b6829, 0x82f63b78, 0x709db87b, 0x63cd4b8f, 0x91a6c88c,
    0x456cac67, 0xb7072f64, 0xa457dc90, 0x563c5f93, 0x082f63b7, 0xfa44e0b4,
    0xe9141340, 0x1b7f9043, 0xcfb5f4a8, 0x3dde77ab, 0x2e8e845f, 0xdce5075c,
    0x92a8fc17, 0x60c37f14, 0x73938ce0, 0x81f80fe3, 0x55326b08, 0xa759e80b,
    0xb4091bff, 0x466298fc, 0x1871a4d8, 0xea1a27db, 0xf94ad42f, 0x0b21572c,
    0xdfeb33c7, 0x2d80b0c4, 0x3ed04330, 0xccbbc033, 0xa24bb5a6, 0x502036a5,
    0x4370c551, 0xb11b4652, 0x65d122b9, 0x97baa1ba, 0x84ea524e, 0x7681d14d,
    0x2892ed69, 0xdaf96e6a, 0xc9a99d9e, 0x3bc21e9d, 0xef087a76, 0x1d63f975,
    0x0e330a81, 0xfc588982, 0xb21572c9, 0x407ef1ca, 0x532e023e, 0xa145813d,
    0x758fe5d6, 0x87e466d5, 0x94b49521, 0x66df1622, 0x38cc2a06, 0xcaa7a905,
    0xd9f75af1, 0x2b9cd9f2, 0xff56bd19, 0x0d3d3e1a, 0x1e6dcdee, 0xec064eed,
    0xc38d26c4, 0x31e6a5c7, 0x22b65633, 0xd0ddd530, 0x0417b1db, 0xf67c32d8,
    0xe52cc12c, 0x1747422f, 0x49547e0b, 0xbb3ffd08, 0xa86f0efc, 0x5a048dff,
    0x8ecee914, 0x7ca56a17, 0x6ff599e3, 0x9d9e1ae0, 0xd3d3e1ab, 0x21b862a8,
    0x32e8915c, 0xc083125f, 0x144976b4, 0xe622f5b7, 0xf5720643, 0x07198540,
    0x590ab964, 0xab613a67, 0xb831c993, 0x4a5a4a90, 0x9e902e7b, 0x6cfbad78,
    0x7fab5e8c, 0x8dc0dd8f, 0xe330a81a, 0x115b2b19, 0x020bd8ed, 0xf0605bee,
    0x24aa3f05, 0xd6c1bc06, 0xc5914ff2, 0x37faccf1, 0x69e9f0d5, 0x9b8273d6,
    0x88d28022, 0x7ab90321, 0xae7367ca, 0x5c18e4c9, 0x4f48173d, 0xbd23943e,
    0xf36e6f75, 0x0105ec76, 0x12551f82, 0xe03e9c81, 0x34f4f86a, 0xc69f7b69,
    0xd5cf889d, 0x27a40b9e, 0x79b737ba, 0x8bdcb4b9, 0x988c474d, 0x6ae7c44e,
    0xbe2da0a5, 0x4c4623a6, 0x5f16d052, 0xad7d5351};
};

}  // namespace airi
}  // namespace crdc
