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

#include "cyber/tools/cyber_visualizer/pointcloud.h"

#include <iostream>

PointCloud::PointCloud(
    int pointCount, int vertexElementCount,
    const std::shared_ptr<QOpenGLShaderProgram>& shaderProgram)
    : RenderableObject(pointCount, vertexElementCount, shaderProgram),
      buffer_(nullptr) {}

PointCloud::~PointCloud(void) {
  if (buffer_) {
    delete[] buffer_;
    buffer_ = nullptr;
  }
}

bool PointCloud::FillVertexBuffer(GLfloat* pBuffer) {
  if (buffer_ && pBuffer) {
    memcpy(pBuffer, buffer_, VertexBufferSize());
    delete[] buffer_;
    buffer_ = nullptr;
    return true;
  } else {
    std::cout << "---Error!!! cannot upload data to Graphics Card----"
              << std::endl;
    return false;
  }
}

bool PointCloud::FillData(
    const std::shared_ptr<crdc::airi::PointCloud2>& pdata) {
  assert(vertex_count() == pdata->point_size());
  buffer_ = new GLfloat[pdata->width() * pdata->height() * vertex_element_count()];
  if (buffer_) {
    GLfloat* tmp = buffer_;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(pdata->data().data());
    uint32_t counter = 0;
    for (auto row = 0; row < pdata->height(); ++row) {
      const uint8_t* row_data = data + row * pdata->point_step();
      for (auto col = 0; col < pdata->width(); ++col) {
        const uint8_t* col_data = row_data + col * pdata->point_step();
        auto x = *(reinterpret_cast<const float*>(col_data + pdata->fields(0).offset()));
        auto y = *(reinterpret_cast<const float*>(col_data + pdata->fields(1).offset()));
        auto z = *(reinterpret_cast<const float*>(col_data + pdata->fields(2).offset()));
        auto i = *(reinterpret_cast<const uint8_t*>(col_data + pdata->fields(3).offset()));
        tmp[counter] = x;
        tmp[counter + 1] = z;
        tmp[counter + 2] = -y;
        tmp[counter + 3] = static_cast<float>(i);
        counter += 4;
      }

    }
    return true;
  }
  return false;
}
