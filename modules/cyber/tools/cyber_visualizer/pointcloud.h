/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>

#include "cyber/sensor_proto/lidar.pb.h"
#include "cyber/tools/cyber_visualizer/renderable_object.h"

class QOpenGLShaderProgram;

struct LidarPoint {
  float x_ = 0;
  float y_ = 0;
  float z_ = 0;
  uint8_t intensity_ = 0;
  float distance = 0;
  uint64_t timestamp_ = 0;
  uint16_t ring_ = 0;
  float azimuth_ = 0;
  float elevation_ = 0;
  uint8_t semantic_flag_ = 0;
  LidarPoint() {}
};

class PointCloud : public RenderableObject {
 public:
  explicit PointCloud(int pointCount = 1, int vertex_element_count = 3,
                      const std::shared_ptr<QOpenGLShaderProgram>&
                          shaderProgram = NullRenderableObj);
  ~PointCloud(void);

  virtual GLenum GetPrimitiveType(void) const { return GL_POINTS; }

  bool FillData(
      const std::shared_ptr<crdc::airi::PointCloud2>& pData);

 private:
  virtual bool FillVertexBuffer(GLfloat* vertexBuffer);

  GLfloat* buffer_;
};
