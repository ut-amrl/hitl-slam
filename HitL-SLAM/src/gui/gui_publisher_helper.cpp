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

#include "gui_publisher_helper.h"

namespace cobot_gui {

void ClearDrawingMessage(cobot_msgs::LidarDisplayMsg* display_msg) {
  display_msg->lines_p1x.clear();
  display_msg->lines_p1y.clear();
  display_msg->lines_p2x.clear();
  display_msg->lines_p2y.clear();
  display_msg->points_x.clear();
  display_msg->points_y.clear();
  display_msg->lines_col.clear();
  display_msg->points_col.clear();
  display_msg->circles_x.clear();
  display_msg->circles_y.clear();
  display_msg->circles_col.clear();
  display_msg->text.clear();
  display_msg->text_x.clear();
  display_msg->text_y.clear();
  display_msg->text_col.clear();
  display_msg->text_in_window_coords.clear();
  display_msg->windowSize = 1.0;
}

}  // namespace cobot_gui
