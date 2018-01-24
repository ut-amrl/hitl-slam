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
/*!
\file    vector_display_main.cpp
\brief   A graphical vector localization vizualizer
\author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <stdio.h>

#include <eigen3/Eigen/Dense>
#include <QtGui/QApplication>
#include <QWidget>

#include "vector_slam_msgs/GuiKeyboardEvent.h"
#include "vector_slam_msgs/GuiMouseClickEvent.h"
#include "vector_slam_msgs/GuiMouseMoveEvent.h"
#include "vector_slam_msgs/LocalizationGuiCaptureSrv.h"
#include "util.h"
#include "geometry.h"
#include "popt_pp.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "../shared/util/helpers.h"
#include "terminal_utils.h"
#include "../shared/util/timer.h"
#include "vector_display.h"
#include "vector_display_thread.h"
//#include "../map/vector_map.h"
#include "configreader.h"

using Eigen::Vector2f;
using ros::Publisher;
using ros::Subscriber;
using std::max;
using std::size_t;
using std::string;
using std::vector;

ros::Publisher mouse_click_publisher_;
ros::Publisher mouse_move_publisher_;
ros::Publisher keyboard_events_publisher_;
ros::ServiceServer capture_service_;

vector_slam_msgs::GuiMouseMoveEvent mouse_move_msg_;
vector_slam_msgs::GuiMouseClickEvent mouse_click_msg_;
vector_slam_msgs::GuiKeyboardEvent keyboard_events_msg_;

VectorDisplayThread* thread_ = NULL;
VectorDisplay* display_ = NULL;

void KeyboardEventCallback(uint32_t key_code, uint32_t modifiers) {
  if (thread_ != NULL)
    thread_->KeyboardEventCallback(key_code, modifiers);
  keyboard_events_msg_.header.seq++;
  keyboard_events_msg_.header.stamp = ros::Time::now();
  keyboard_events_msg_.keycode = key_code;
  keyboard_events_msg_.modifiers = modifiers;
  keyboard_events_publisher_.publish(keyboard_events_msg_);
}

void MouseMoveCallback(
    const Vector2f& location, uint32_t buttons, uint32_t modifiers) {
  mouse_move_msg_.location.x = location.x();
  mouse_move_msg_.location.y = location.y();
  mouse_move_msg_.buttons = buttons;
  mouse_move_msg_.modifiers = modifiers;
  mouse_move_publisher_.publish(mouse_move_msg_);
}

void MouseClickCallback(
    const Vector2f& mouse_down, const Vector2f& mouse_up, float orientation,
    uint32_t modifiers) {
  if (thread_ != NULL) {
    thread_->MouseEventCallback(mouse_down, mouse_up, orientation, modifiers);
  }
  mouse_click_msg_.header.seq++;
  mouse_click_msg_.header.stamp = ros::Time::now();
  mouse_click_msg_.modifiers = modifiers;
  mouse_click_msg_.mouse_down.x = mouse_down.x();
  mouse_click_msg_.mouse_down.y = mouse_down.y();
  mouse_click_msg_.mouse_up.x = mouse_up.x();
  mouse_click_msg_.mouse_up.y = mouse_up.y();
  mouse_click_publisher_.publish(mouse_click_msg_);
}

bool CaptureCallback(vector_slam_msgs::LocalizationGuiCaptureSrv::Request& req,
                     vector_slam_msgs::LocalizationGuiCaptureSrv::Response& res) {
  printf("Saving image to %s\n", req.filename.c_str());
  if (display_ != NULL) {
    display_->Capture(req.filename);
  }
  return true;
}

int main(int argc, char *argv[]) {

  QApplication* app = NULL;

  bool testMode = false;
  char* map_option = NULL;
  bool saveLocs = false;
  bool liveView = false;
  bool persistentDisplay = true;
  bool savePoses = false;
  bool blankDisplay = false;
  // Maximum display refresh rate.
  float maxFps = 60.0;

  static const bool debug = false;
  ColourTerminal(TerminalUtils::TERMINAL_COL_GREEN,
      TerminalUtils::TERMINAL_COL_BLACK,
      TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("\nVector Localization GUI\n\n");
  ResetTerminal();

  if (debug) printf("Starting up...\n");

  const string node_name = StringPrintf(
      "vector_display_%lu",
      static_cast<uint64_t>(GetTimeSec() * 1000.0));
  ros::init(argc, argv, node_name);

  // option table
  double window_location_x = FLT_MAX, window_location_y = FLT_MAX;
  double window_size = 100.0;
  bool edit_map = false;
  bool nav_map = false;
  bool semantic_map = false;
  bool semantic_view = false;
  bool nav_view = false;
  bool vector_file = false;
  static struct poptOption options[] = {
    { "test-mode", 't', POPT_ARG_NONE , &testMode, 0,
        "Test Drawing Capabilities", "NONE"},
    { "map-name", 'm', POPT_ARG_STRING , &map_option, 0, "Map name", "STRING"},
    { "save-lines", 's', POPT_ARG_NONE, &saveLocs, 0, "Save lines", "NONE"},
    { "live-view", 'l', POPT_ARG_NONE, &liveView, 0, "Live View of Robot",
        "NONE"},
    { "persistent-display", 'p', POPT_ARG_NONE, &persistentDisplay, 0,
        "Persistent Display", "NONE"},
    { "save-poses", 'S', POPT_ARG_NONE, &savePoses, 0,
        "Save location and angle poses", "NONE"},
    { "blank-display", 'b', POPT_ARG_NONE, &blankDisplay, 0,
        "Blank Display", "NONE"},
    { "location-x", 'x', POPT_ARG_DOUBLE, &window_location_x, 0,
        "Window location X", "NONE"},
    { "location-y", 'y', POPT_ARG_DOUBLE, &window_location_y, 0,
        "Window location Y", "NONE"},
    { "window-size", 'z', POPT_ARG_DOUBLE, &window_size, 0,
        "Window size", "NONE"},
    { "max-fps", 'f', POPT_ARG_FLOAT, &maxFps, 0,
        "maximum display refresh rate", "NONE"},
    { "edit-map", 'e', POPT_ARG_NONE, &edit_map, 0,
        "Edit Map", "NONE"},
    { "edit-nav-map", 'n', POPT_ARG_NONE, &nav_map, 0,
        "Edit navigation map", "NONE"},
    { "edit-semantic-map", 'o', POPT_ARG_NONE, &semantic_map, 0,
        "Edit semantic map", "NONE"},
    { "view-semantic-vertices", 'v', POPT_ARG_NONE, &semantic_view, 0,
        "View semantic vertices", "NONE"},
    { "view-navigation-map", 'g', POPT_ARG_NONE, &nav_view, 0,
        "View navigation map", "NONE"},
    { "view-vector-file", 'V', POPT_ARG_NONE, &vector_file, 0,
        "View vector file", "NONE"},
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  // parse options
  POpt popt(NULL, argc, (const char**)argv, options, 0);
  int c;
  while ((c = popt.getNextOpt()) >= 0) {
  }
  if (savePoses)
    saveLocs = true;

  mouse_click_msg_.header.frame_id = "map";
  mouse_click_msg_.header.seq = 0;
  mouse_click_msg_.header.stamp.fromSec(0.0);
  mouse_click_msg_.mouse_down.z = 0.0;
  mouse_click_msg_.mouse_up.z = 0.0;

  mouse_move_msg_.header.frame_id = "map";
  mouse_move_msg_.header.seq = 0;
  mouse_move_msg_.header.stamp.fromSec(0.0);
  mouse_move_msg_.location.z = 0.0;

  keyboard_events_msg_.header.frame_id = "map";
  keyboard_events_msg_.header.seq = 0;
  keyboard_events_msg_.header.stamp.fromSec(0.0);

  ros::NodeHandle node_handle;
  mouse_move_publisher_ =
      node_handle.advertise<vector_slam_msgs::GuiMouseMoveEvent>(
      "VectorSLAM/VectorLocalization/GuiMouseMoveEvents", 1, false);
  mouse_click_publisher_ =
      node_handle.advertise<vector_slam_msgs::GuiMouseClickEvent>(
      "VectorSLAM/VectorLocalization/GuiMouseClickEvents", 1, false);
  keyboard_events_publisher_ =
      node_handle.advertise<vector_slam_msgs::GuiKeyboardEvent>(
      "VectorSLAM/VectorLocalization/GuiKeyboardEvents", 1, false);
  capture_service_ = node_handle.advertiseService(
      "VectorLocalization/Capture", CaptureCallback);

  app = new QApplication(argc, argv);
  display_ = new VectorDisplay();

  //InitHandleStop(&runApp);
  display_->show();
  const string mapsFolder =
      ros::package::getPath("cobot_linux") + "/../maps";
  thread_ = new VectorDisplayThread(mapsFolder, display_, &node_handle, app);
  const string map_name = (map_option == NULL) ? "LGRC3" : string(map_option);
  thread_->setOptions(testMode, map_name, saveLocs, liveView, persistentDisplay,
      savePoses, blankDisplay, maxFps, nav_map, semantic_map,
      semantic_view, nav_view, vector_file);

  display_->setMouseClickCallback(&MouseClickCallback);
  display_->setKeyboardCallback(&KeyboardEventCallback);
  display_->setMouseMoveCallback(&MouseMoveCallback);

  thread_->start();

  if (window_location_x != FLT_MAX || window_location_y != FLT_MAX) {
    if (window_location_x == FLT_MAX) window_location_x = 0.0;
    if (window_location_y == FLT_MAX) window_location_y = 0.0;
  } else {
    ConfigReader config_((ros::package::getPath("cobot_linux") + "/").c_str());
    config_.init();
    config_.addFile("../robot.cfg");
    config_.addFile("config/non_markov_localization.cfg");
    if (!config_.readFiles()) return false;
    ConfigReader::SubTree c(config_,"NonMarkovLocalization");
    bool error = false;
    error = error || !c.getReal("starting_location.x", window_location_x);
    error = error || !c.getReal("starting_location.y", window_location_y);
    if (error) {
      window_location_x = 0.0;
      window_location_y = 0.0;
    }
  }

  display_->setView(window_location_x, window_location_y, window_size);
  int retVal = app->exec();
  thread_->setRunApp(false);
  if (debug) printf("Waiting for thread termination... ");
  thread_->wait();
  if (debug) printf("Done. Bye Bye!\n");
  delete thread_;
  delete display_;
  return retVal;
}
