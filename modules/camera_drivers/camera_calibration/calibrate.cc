#include "camera_drivers/camera_calibration/calibrate.h"
#include "framework/internal/Global.h"

namespace crdc {
namespace airi {

void Calibrate::init(const std::string &config_path,
                     const int img_width, const int img_height) {
  // init calibration
  camera_calib_ = std::make_shared<CameraCalibrate>();
  calib_input_data_ = std::make_shared<DbusSendData>();
  camera_calib_->init(config_path, img_width, img_height);
  process_state_ = 1;
  vehicle_mode_ = 0;
  image_count_ = 0;
  begin();
  product_name_ = get_product_name();
}
void Calibrate::begin() {
  connection = IpcConnection::getConnection(
      IpcConnection::IPC_TYPE_DBUS, String8::format("%s", "CameraDrivers"));
  Global::setMainConnection(connection);
  status_server_ = new CalibrateStatus();
  std::string path = camera_calib_->get_params_file_path();
  status_server_->set_calibrate_path(path);
  connection->addService(status_server_);
  connection->start();
  status_server_->start();
}

int Calibrate::get_vehicle_mode() {
  int request_id = 0;
  status_server_ -> get_request_id(request_id);
  vehicle_mode_ = request_id;
  return vehicle_mode_;
}

int Calibrate::process(std::shared_ptr<const CameraRawData> &images_raw,
                       std::shared_ptr<CamIntrinsicParam> &camera_intrinsic) {
  int calib_state;
  if (calib_end_) {
    calib_end_ = false;
    process_state_ = 1;
    status_server_->set_status(process_state_);
  }
  if (process_state_ != 3) {
    // preparing input of calibration
    process_state_ = 1;
    status_server_->set_status(process_state_);
    image_count_++;
     if (image_count_ % 10 == 0) {
      images_.push_back(images_raw);
    }
    if (images_.size() < 5) {
      LOG(INFO) << "No enough input images";
      return 0;
    }
    calib_input_data_ = status_server_->extracted_calib_param();
    // start calibration process
    process_state_ = 2;
    status_server_->set_status(process_state_);
    if (calib_input_data_->calib_mode == 2) {
      camera_calib_->init_instrinsic_distortion_coeffs(camera_intrinsic);

      if (camera_calib_->cal_world_coord(calib_input_data_->rotation_vector,
                                         calib_input_data_->translation_vector)) {
        camera_calib_->generate_real_distance_lut();
      }
      calib_state = 0;
    } else {
      calib_state =
          camera_calib_->process(images_, save_path_, calib_input_data_, camera_intrinsic);
    }
    if (!calib_state) {
      // send response
      std::vector<std::shared_ptr<const CameraRawData>>().swap(images_);
      process_state_ = 3;
      status_server_->set_status(process_state_);
      status_server_->reset_caliration_state();
      return 0;
    } else {
      std::vector<std::shared_ptr<const CameraRawData>>().swap(images_);
      LOG(INFO) << "Calibration failed. Try to calibrate again.";
      process_state_ = 4;
      status_server_->set_status(process_state_);
      status_server_->reset_caliration_state();
      return 0;
    }
  } else {
    return 1;
  }
}

}  // namespace airi
}  // namespace crdc


