//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
// Copyright 2013 joydeepb@ri.cmu.edu
// Robotics Institute, Carnegie Mellon University
//
// Helper Functions to draw primitives on the CoBot localization GUI.

#include <eigen3/Eigen/Dense>
#include <vector>
#include "cobot_msgs/LidarDisplayMsg.h"

#ifndef GUI_PUBLISHER_HELPER_H
#define GUI_PUBLISHER_HELPER_H

namespace cobot_gui {

// Some commonly used colors.
const uint32_t kColorRed = 0xFFFF0000;
const uint32_t kColorGreen = 0xFF00FF00;
const uint32_t kColorBlue = 0xFF0000FF;
const uint32_t kColorYellow = 0xFFFFFF00;
const uint32_t kColorBlack = 0xFF000000;
const uint32_t kColorWhite = 0xFFFFFFFF;

template <typename VectorType>
void DrawCircle(const VectorType& p, uint32_t color,
                cobot_msgs::LidarDisplayMsg* display_msg) {
  display_msg->circles_x.push_back(p.x);
  display_msg->circles_y.push_back(p.y);
  display_msg->circles_col.push_back(color);
}

template <typename VectorType>
void DrawLine(const VectorType& p0, const VectorType& p1, uint32_t color,
              cobot_msgs::LidarDisplayMsg* display_msg) {
  display_msg->lines_p1x.push_back(p0.x);
  display_msg->lines_p1y.push_back(p0.y);
  display_msg->lines_p2x.push_back(p1.x);
  display_msg->lines_p2y.push_back(p1.y);
  display_msg->lines_col.push_back(color);
}

template <typename ScalarType>
void DrawLine(const Eigen::Matrix<ScalarType, 2, 1>& p0,
              const Eigen::Matrix<ScalarType, 2, 1>& p1, uint32_t color,
              cobot_msgs::LidarDisplayMsg* display_msg) {
  display_msg->lines_p1x.push_back(p0.x());
  display_msg->lines_p1y.push_back(p0.y());
  display_msg->lines_p2x.push_back(p1.x());
  display_msg->lines_p2y.push_back(p1.y());
  display_msg->lines_col.push_back(color);
}

template <typename VectorType>
void DrawPoint(const VectorType& p, uint32_t color,
               cobot_msgs::LidarDisplayMsg* display_msg) {
  display_msg->points_x.push_back(p.x);
  display_msg->points_y.push_back(p.y);
  display_msg->points_col.push_back(color);
}

template <typename ScalarType>
void DrawPoint(const Eigen::Matrix<ScalarType, 2, 1>& p,
               uint32_t color,
               cobot_msgs::LidarDisplayMsg* display_msg) {
  display_msg->points_x.push_back(p.x());
  display_msg->points_y.push_back(p.y());
  display_msg->points_col.push_back(color);
}

template <typename VectorType>
void DrawText(const VectorType& loc,
              const std::string& text,
              uint32_t color,
              float text_height,
              bool text_in_window_coords,
              cobot_msgs::LidarDisplayMsg* display_msg) {
  display_msg->text_x.push_back(loc.x);
  display_msg->text_y.push_back(loc.y);
  display_msg->text_col.push_back(color);
  display_msg->text.push_back(text);
  display_msg->text_height.push_back(text_height);
  display_msg->text_in_window_coords.push_back(text_in_window_coords);
}

template <typename ScalarType>
void DrawText(const Eigen::Matrix<ScalarType, 2, 1>& loc,
              const std::string& text,
              uint32_t color,
              float text_height,
              bool text_in_window_coords,
              cobot_msgs::LidarDisplayMsg* display_msg) {
  display_msg->text_x.push_back(loc.x());
  display_msg->text_y.push_back(loc.y());
  display_msg->text_col.push_back(color);
  display_msg->text.push_back(text);
  display_msg->text_height.push_back(text_height);
  display_msg->text_in_window_coords.push_back(text_in_window_coords);
}

void ClearDrawingMessage(cobot_msgs::LidarDisplayMsg* display_msg);

}  // namespace cobot_gui

#endif  // GUI_PUBLISHER_HELPER_H
