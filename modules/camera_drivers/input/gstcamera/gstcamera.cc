// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera input gstcamera

#include "camera_drivers/input/gstcamera/gstcamera.h"

namespace crdc {
namespace airi {

GstCamera::~GstCamera() {
  if (is_running_.load()) {
    stop();
    LOG(INFO) << "[" << config_.frame_id() << "] camera stoped.";
  }

  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = NULL;
  }

  if (sink_) {
    gst_object_unref(sink_);
    sink_ = NULL;
  }
}

bool GstCamera::camera_init() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input init.";
  if (!gst_is_initialized()) {
    // Initialize gstreamer pipeline
    LOG(INFO) << "[" << config_.frame_id() << "] Initializing gstreamer.";
    gst_init(0, 0);
  }

  if (!config_.has_gst_config()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] config type error.";
    return false;
  }

  LOG(INFO) << "[" << config_.frame_id() << "] Gstreamer Version: " << gst_version_string();

  GError *gst_error = 0;  // Assignment to zero is a gst requirement

  gsconfig_ = config_.gst_config().gst_launch();

  LOG(INFO) << "[" << config_.frame_id() << "] gsconfig_: " << gsconfig_;

  pipeline_ = gst_parse_launch(gsconfig_.c_str(), &gst_error);
  if (pipeline_ == NULL) {
    LOG(ERROR) << "[" << config_.frame_id() << "] gstreamer error domain: "<< gst_error->domain
              << ", code: " << gst_error->code << ", msg: " << gst_error->message;
    return false;
  }

  // Create RGB sink
  sink_ = gst_element_factory_make("appsink", NULL);
  gst_app_sink_set_max_buffers(reinterpret_cast<GstAppSink*>(sink_), 1);
  gst_app_sink_set_drop(reinterpret_cast<GstAppSink*>(sink_), true);
  GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));
  caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  if (GST_IS_PIPELINE(pipeline_)) {
    GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    g_assert(outpad);

    GstElement *outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);

    if (!gst_bin_add(GST_BIN(pipeline_), sink_)) {
      LOG(ERROR) << "[" << config_.frame_id() << "] gst_bin_add() failed.";
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }

    if (!gst_element_link(outelement, sink_)) {
      LOG(ERROR) << "[" << config_.frame_id() << "] GStreamer: Appsink cannot link outelement.";
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }
    gst_object_unref(outelement);
  } else {
    GstElement* launchpipe = pipeline_;
    pipeline_ = gst_pipeline_new(NULL);
    g_assert(pipeline_);
    gst_object_unparent(GST_OBJECT(launchpipe));
    gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);
    if (!gst_element_link(launchpipe, sink_)) {
      LOG(ERROR) << "[" << config_.frame_id() << "] GStreamer: cannot link launchpipe -> sink.";
      gst_object_unref(pipeline_);
      return false;
    }
  }

  gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to PAUSE stream.";
      return false;
  } else {
      LOG(INFO) << "[" << config_.frame_id() << "] Stream is PAUSED.";
  }
  return true;
}

bool GstCamera::camera_start() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input start.";
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    LOG(ERROR) << "[" << config_.frame_id() << "] Could not start stream!";
    return false;
  }
  LOG(INFO) << "[" << config_.frame_id() << "] Started stream.";
  auto data_thread = new std::thread(&GstCamera::get_data, this);
  return true;
}

bool GstCamera::camera_stop() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input stopped.";
  gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to PAUSE stream.";
      return false;
  } else {
      LOG(INFO) << "[" << config_.frame_id() << "] Stream is PAUSED.";
  }
  return true;
}

void GstCamera::get_data() {
  while (1) {
    unsigned char* data;
    uint32_t data_size = 0;
#if (GST_VERSION_MAJOR == 1)
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    if (!sample) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Could not get gstreamer sample.";
      break;
    }
    GstBuffer* buf = gst_sample_get_buffer(sample);
    GstMapInfo info;
    if (buf != nullptr) {
      gst_buffer_map(buf, &info, GST_MAP_READ);
      data = info.data;
      data_size = info.size;
    }
#else
    GstBuffer* buf = gst_app_sink_pull_buffer(GST_APP_SINK(sink_));
    // Stop on end of stream
    if (buf != nullptr) {
      data = buf->data;
      data_size = buf->size;
    }
#endif
    LOG(INFO) << "[" << config_.frame_id() << "] buf_size:" + std::to_string(data_size);
    // Get the image width and height
    GstPad* pad = gst_element_get_static_pad(sink_, "sink");

#if (GST_VERSION_MAJOR == 1)
    const GstCaps *caps = gst_pad_get_current_caps(pad);
#else
    const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
#endif
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &width_);
    gst_structure_get_int(structure, "height", &height_);
    LOG(INFO) << "[" << config_.frame_id() << "] image->width height:" << width_ << " " << height_;

    std::shared_ptr<CameraRawData> raw_data = get_raw_data(0, get_now_microsecond(), data);
    raw_data->data_type = "JPG";
    raw_data->data_size = data_size;
    LOG(INFO) << "[" << config_.frame_id() << "] image->width height:" << raw_data->image_.cols
              << " " << raw_data->image_.rows;
    raw_data_queue_.enqueue(raw_data);
    if (buf) {
#if (GST_VERSION_MAJOR == 1)
    gst_buffer_unmap(buf, &info);
    gst_sample_unref(sample);
#else
    gst_buffer_unref(buf);
#endif
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / config_.fps()));
  }
}

}  // namespace airi
}  // namespace crdc
