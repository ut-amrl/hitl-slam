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
\file    vector_display_thread.h
\brief   Thread to run the GUI for Vector Localization; C++ Implementation: VectorDisplayThread
\author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#ifndef VECTOR_DISPLAY_THREAD_H_
#define VECTOR_DISPLAY_THREAD_H_

#include <stdio.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <QtGui/QApplication>
#include <QWidget>
#include <QObject>
#include <QThread>
#include <string>
#include <vector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include "cobot_msgs/LidarDisplayMsg.h"
#include "cobot_msgs/CobotHumansDetected.h"
#include "cobot_msgs/CobotHumansClassified.h"
#include "cobot_msgs/CobotRemoteInterfaceSrv.h"
#include "cobot_msgs/CobotStatusMsg.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "cobot_msgs/CobotAnomalyMonitorRectangleMsg.h"
#include "vector_display.h"
#include "../map/vector_map.h"
#include "../map/navigation_map.h"

class ScopedFile;

class VectorDisplayThread : public QThread {
  // Q_OBJECT

private:

  ros::NodeHandle *node_handle_;

  bool runApp;
  bool testMode;
  bool mapEditMode;
  bool navMapMode;
  bool semanticMapMode;
  bool semanticViewMode;
  bool navViewMode;
  bool viewVectorFile;
  bool liveView;
  bool persistentDisplay;
  bool blankDisplay;
  bool showAnomalyProb;
  bool saveLocs;
  bool saveOrientations;
  bool clearDisplay;
  bool autoUpdateMap;
  // Maximum display refresh rate.
  float maxFps;

  geometry_msgs::PoseWithCovarianceStamped localizationInitMsg;
  ros::Publisher initialPosePublisher;
  ros::ServiceClient client;
  ros::ServiceClient localizationClient;
  ros::ServiceClient managerClient;
  ros::ServiceClient autoLocalizeClient;

  QApplication* app;
  VectorDisplay* display;
  std::string map_name_;

  NavigationMap navMap;

  std::string mapsFolder;
  VectorMap vectorMap;
  std::vector<Eigen::Vector2f> pathPlan;
  Eigen::Vector2f robotLoc;
  float robotAngle;
  float anomalyLLX;
  float anomalyLLY;
  float anomalyURX;
  float anomalyURY;
  float anomaly;
  cobot_msgs::CobotHumansDetected humansMsg;
  cobot_msgs::CobotHumansClassified classifiedHumansMsg;
  sensor_msgs::LaserScan laserScanMsg;
  sensor_msgs::LaserScan kinectScanMsg;
  sensor_msgs::PointCloud pointCloudMsg;
  std::vector<cobot_msgs::LidarDisplayMsg> displayMsgs;
  std::vector<std::string> displayProviders;
  double tPathPlan, tLaser, tPointCloud, tHumanDetect, tHumanTrack;

  std::vector<VectorDisplay::Line> lines;
  std::vector<Eigen::Vector2f> points;
  std::vector<Eigen::Vector2f> circles;
  std::vector<VectorDisplay::Quad> quads;
  std::vector<VectorDisplay::Color> circleColors;
  std::vector<VectorDisplay::Color> lineColors;
  std::vector<VectorDisplay::Color> pointColors;
  std::vector<VectorDisplay::Color> quadColors;

  std::vector<std::string> textStrings;
  std::vector<Eigen::Vector2f> textLocs;
  std::vector<float> textHeights;
  std::vector<VectorDisplay::Color> textColors;
  std::vector<bool> textInWindowCoords;

  ScopedFile* saveLocsFile;

public:

  void Zoom(float zoom);

  std::string GetMapName() { return map_name_; }

  bool GetNavEdgeParams(
      float* width,
      float* max_speed,
      bool* has_door);

  bool GetSemanticType(
      const std::vector<std::string>& types,
      std::string* selected_type);

  bool GetSemanticTypeAndLabel(
      const std::vector<std::string>& types,
      std::string* selected_type,
      std::string* label);

  void ChangeMap();

  void AutoLocalize();

  void KeyboardEventCallback(uint32_t key_code, uint32_t modifiers);

  void MouseEventCallback(
      const Eigen::Vector2f& mouse_down,
      const Eigen::Vector2f& mouse_up, float orientation,
      uint32_t modifiers);

  void drawMap(std::vector<VectorDisplay::Line>* lines,
               std::vector<VectorDisplay::Color>* lineColors);

  void cobotLocalizationCallback(
      const cobot_msgs::CobotLocalizationMsg& msg);

  void cobotAnomalyCallback(
      const cobot_msgs::CobotAnomalyMonitorRectangleMsg& msg);

  void kinectScanCallback(const sensor_msgs::LaserScan& msg);

  void laserCallback(const sensor_msgs::LaserScan& msg);

  void cobotStatusCallback(const cobot_msgs::CobotStatusMsg& msg);

  void humanDetectionCallback(const cobot_msgs::CobotHumansDetected& msg);

  void humanTrackingCallback(const cobot_msgs::CobotHumansClassified& msg);

  void statusCallback(
      const ros::MessageEvent<const cobot_msgs::LidarDisplayMsg>& msgEvent);

  void filteredPointCloudCallback(const sensor_msgs::PointCloud& msg);

  void clearDisplayMessages();

  void compileDisplay();

  void setRunApp(bool newRunApp) {
    runApp = newRunApp;
  }

  void setLiveView(bool liveView) {
    this->liveView = liveView;
  }

  void setOptions(
      bool testMode, std::string mapName, bool saveLocs, bool liveView,
      bool persistentDisplay, bool saveOrientations, bool blankDisplay,
      float maxFps, bool mapEditMode, bool navMapMode, bool semanticMapMode,
      bool semanticViewMode, bool navViewMode, bool viewVectorFile);

protected:
  void run();
  // Edit the localization map
  void editMap(const Eigen::Vector2f& mouse_down,
               const Eigen::Vector2f& mouse_up, float orientation,
               uint32_t modifiers);
  // Edit the navigation map or semantic map (both are really graphs)
  void editGraph(const Eigen::Vector2f& mouse_down,
                 const Eigen::Vector2f& mouse_up, float orientation,
                 uint32_t modifiers);

public:
  VectorDisplayThread(
      const std::string& _mapsFolder, VectorDisplay* disp,
      ros::NodeHandle* node_handle, QApplication* qapp = 0,
      QObject* parent = 0);
  ~VectorDisplayThread();
};

#endif /* VECTOR_DISPLAY_THREAD_H_ */
