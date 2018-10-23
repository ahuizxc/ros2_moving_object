/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>
#include "object_analytics_node/movement/param.hpp"

namespace object_analytics_node
{
namespace movement
{

Param::Param()
{
  init();
}

void Param::init()
{
  social_filtering_enabled_ = kObjectFiltering_TypeFiltering_Enabling;
  moving_object_msg_enabled_ = true; // should be true.
  posibility_threshold_ = kObjectFiltering_PosibilityThreshold;
  max_frames_ = kFrameCache_Size;
  velocity_enabled_ = kVelocityCalculation_Enabling;
  fixed_frame_ = kVelocityCalculation_FixedFrame;
  msg_object_detection_ = kMessage_ObjectDetection;
  msg_object_tracking_ = kMessage_ObjectTracking;
  msg_object_localization_ = kMessage_ObjectLocalization;
}

bool Param::validateParam()
{
  /**< TODO todo: Add validation criteria here. */

  return true;
}

}  // namespace movement
}  // namespace object_analytics_node