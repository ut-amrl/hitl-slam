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
\file    py_vector_display.cpp
\brief   A python interface for the Vector Display
\author  Michael Murphy
 */
//========================================================================

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <stdio.h>

#include "cobot_msgs/CobotLocalizationSrv.h"
#include "cobot_msgs/CobotRemoteInterfaceSrv.h"
#include "cobot_msgs/GuiMouseClickEvent.h"
#include "vector_display.h"
#include "vector_display_thread.h"
#include "py_vector_display.h"
#include "configreader.h"

using Eigen::Vector2f;
using cobot_msgs::CobotLocalizationSrv;
using cobot_msgs::CobotRemoteInterfaceSrv;
using std::size_t;
using std::string;
using std::vector;

namespace {

enum InteractionMode {
  kInteractionPan = 0,
  kInteractionZoom,
  kInteractionLocalize,
  kInteractionGoto,
  kInteractionClick,
};

}  // namespace

bool initialized_ = false;
VectorDisplay* display_ = NULL;
VectorDisplayThread* thread_ = NULL;
ros::NodeHandle* PyVectorDisplay::node_handle_ = NULL;
ros::ServiceClient auto_localization_client_;
ros::ServiceClient localization_client_;
ros::Publisher mouse_click_publisher_;
ros::ServiceClient manager_client_;

cobot_msgs::GuiMouseClickEvent mouse_click_msg_;
InteractionMode interaction_mode_ = kInteractionPan;

void KeyboardEventCallback(uint32_t key_code, uint32_t modifiers) {
  if (thread_ != NULL)
    thread_->KeyboardEventCallback(key_code, modifiers);
}

void MouseClickCallback(
      const Vector2f& mouse_down, const Vector2f& mouse_up, float orientation,
      uint32_t modifier) {
  if (thread_ == NULL) return;
  switch (interaction_mode_) {
    case kInteractionPan :
    case kInteractionZoom : {
      // No need to do anything.
    } break;

    case kInteractionLocalize : {
      // Send set location command.
      // Send set target command.
      CobotRemoteInterfaceSrv srv;
      srv.request.loc_x = mouse_down.x();
      srv.request.loc_y = mouse_down.y();
      srv.request.orientation = orientation;
      srv.request.command_num = 0;
      srv.request.map = thread_->GetMapName();
      srv.request.command_type = 0x0002;
      localization_client_.call(srv);
    } break;

    case kInteractionGoto : {
      // Send set target command.
      CobotRemoteInterfaceSrv srv;
      srv.request.loc_x = mouse_down.x();
      srv.request.loc_y = mouse_down.y();
      srv.request.orientation = orientation;
      srv.request.command_num = 0;
      srv.request.map = thread_->GetMapName();
      srv.request.command_type = 0x0020;
      manager_client_.call(srv);
    } break;

    case kInteractionClick : {
      mouse_click_msg_.header.seq++;
      mouse_click_msg_.header.stamp = ros::Time::now();
      mouse_click_msg_.modifiers = modifier;
      mouse_click_msg_.mouse_down.x = mouse_down.x();
      mouse_click_msg_.mouse_down.y = mouse_down.y();
      mouse_click_msg_.mouse_up.x = mouse_up.x();
      mouse_click_msg_.mouse_up.y = mouse_up.y();
      mouse_click_publisher_.publish(mouse_click_msg_);
    } break;

    default : {
      fprintf(stderr, "Unexpected interaction mode in %s:%d\n",
              __FILE__, __LINE__);
    } break;
  }
}

void PyVectorDisplay::DisableDrawing() {
  display_->EnableDrawing(false);
}

void PyVectorDisplay::EnableDrawing() {
  display_->EnableDrawing(true);
}


void PyVectorDisplay::ZoomIn() {
  display_->Zoom(0.1);
}

void PyVectorDisplay::ZoomOut() {
  display_->Zoom(-0.1);
}

void PyVectorDisplay::ChangeMap() {
  thread_->ChangeMap();
}

void PyVectorDisplay::SetInteractionPan() {
  display_->SetInteractionMode(VectorDisplay::kInteractionPan);
  interaction_mode_ = kInteractionPan;
}

void PyVectorDisplay::SetInteractionZoom() {
  display_->SetInteractionMode(VectorDisplay::kInteractionZoom);
  interaction_mode_ = kInteractionZoom;
}

void PyVectorDisplay::SetInteractionLocalize() {
  display_->SetInteractionMode(VectorDisplay::kInteractionSelect);
  interaction_mode_ = kInteractionLocalize;
}

void PyVectorDisplay::SetInteractionGoto() {
  display_->SetInteractionMode(VectorDisplay::kInteractionSelect);
  interaction_mode_ = kInteractionGoto;
}

void PyVectorDisplay::SetInteractionClick() {
  display_->SetInteractionMode(VectorDisplay::kInteractionSelect);
  interaction_mode_ = kInteractionClick;
}

void PyVectorDisplay::AutoLocalize(double x, double y, double theta) {
  CobotLocalizationSrv::Request req;
  CobotLocalizationSrv::Response res;
  req.x = x;
  req.y = y;
  req.angle = theta;
  auto_localization_client_.call(req, res);
}

QWidget* PyVectorDisplay::initializeForPythonUI() {

  if (initialized_)
    PyVectorDisplay::shutdown();

  int fakeargc = 1;
  char* fakeargv[] = {(char*) "", NULL};
  ros::init(fakeargc, fakeargv, "Cobot_UI_Localization_GUI");
  display_ = new VectorDisplay();

  // Center the localization gui on the robot's default starting location
  float window_location_x = 0.0;
  float window_location_y = 0.0;
  ConfigReader config_((ros::package::getPath("cobot_linux") + "/").c_str());
  config_.init();
  config_.addFile("../robot.cfg");
  config_.addFile("config/non_markov_localization.cfg");
  if (config_.readFiles()) {
    ConfigReader::SubTree c(config_,"NonMarkovLocalization");
    bool error = false;
    error = error || !c.getReal("starting_location.x", window_location_x);
    error = error || !c.getReal("starting_location.y", window_location_y);
    if (!error) {
      // Widget may be resized by cobot_ui
      display_->setView(window_location_x, window_location_y, 10);
    }
  }

  node_handle_ = new ros::NodeHandle();

  mouse_click_msg_.header.frame_id = "map";
  mouse_click_msg_.header.seq = 0;
  mouse_click_msg_.header.stamp.fromSec(0.0);
  mouse_click_msg_.mouse_down.z = 0.0;
  mouse_click_msg_.mouse_up.z = 0.0;

  mouse_click_publisher_ =
      node_handle_->advertise<cobot_msgs::GuiMouseClickEvent>(
      "Cobot/VectorLocalization/GuiMouseClickEvents", 1, false);
  localization_client_ = node_handle_->serviceClient<CobotRemoteInterfaceSrv>(
      "Cobot/VectorLocalization/RemoteInterface");
  manager_client_ = node_handle_->serviceClient<CobotRemoteInterfaceSrv>(
      "Cobot/RemoteInterface");
  auto_localization_client_ =
      node_handle_->serviceClient<CobotLocalizationSrv>(
      "Cobot/VectorLocalization/AutoLocalize");
  const string mapsFolder =
      ros::package::getPath("cobot_linux").append("/../maps");
  thread_ = new VectorDisplayThread(mapsFolder, display_, node_handle_);
  thread_->setLiveView(true);
  display_->SetInteractionMode(VectorDisplay::kInteractionPan);
  display_->setMouseClickCallback(&MouseClickCallback);
  display_->setKeyboardCallback(&KeyboardEventCallback);

  thread_->start();

  initialized_ = true;

  QWidget* ret = display_;
  return ret;
}

void PyVectorDisplay::shutdown() {

  if (initialized_) {
    thread_->setRunApp(false); //tells thread to exit
    thread_->wait();
    thread_->deleteLater();
    thread_ = NULL;
    display_->deleteLater();
    display_ = NULL;
    ros::shutdown();
    initialized_ = false;
  }
}
