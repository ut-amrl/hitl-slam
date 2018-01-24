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
\file    vector_display_thread.cpp
\brief   Thread to run the GUI for Vector Localization; C++ Interface: VectorDisplayThread
\author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <string>
#include <vector>
#include <QInputDialog>
#include <QMessageBox>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "vector_slam_msgs/CobotAnomalyMonitorRectangleMsg.h"
//#include "vector_slam_msgs/CobotHumansDetected.h"
#include "vector_slam_msgs/CobotLocalizationMsg.h"
#include "vector_slam_msgs/CobotLocalizationSrv.h"
#include "vector_slam_msgs/CobotRemoteInterfaceSrv.h"
#include "vector_slam_msgs/CobotStatusMsg.h"
#include "vector_slam_msgs/LidarDisplayMsg.h"
#include "vector_display.h"
#include "vector_display_thread.h"
#include "../shared/math/geometry.h"


#include "helpers.h"
#include "proghelp.h"
#include "timer.h"
#include "util.h"

using Eigen::Vector2f;
using Eigen::Rotation2Df;
using vector_slam_msgs::CobotAnomalyMonitorRectangleMsg;
//using vector_slam_msgs::CobotHumansDetected;
using vector_slam_msgs::CobotLocalizationMsg;
using vector_slam_msgs::CobotLocalizationSrv;
using vector_slam_msgs::CobotRemoteInterfaceSrv;
using vector_slam_msgs::CobotStatusMsg;
using vector_slam_msgs::LidarDisplayMsg;
using geometry_msgs::PoseWithCovarianceStamped;
using std::size_t;
using std::string;
using std::vector;

bool VectorDisplayThread::GetSemanticType(
    const vector<string>& types,
    string* selected_type) {
  QStringList types_list;
  int old_type = 0;
  for (size_t i = 0; i < types.size(); ++i) {
    types_list << tr(types[i].c_str());
    if (types[i] == (*selected_type)) {
      old_type = i;
    }
  }
  bool ok = false;
  *selected_type = QInputDialog::getItem(
      display,
      tr("Semantic Type"),
      tr("Type:"),
      types_list,
      old_type,
      false,
      &ok).toStdString();
  return ok;
}

bool VectorDisplayThread::GetSemanticTypeAndLabel(
    const vector<string>& types,
    string* selected_type,
    string* label) {
  if (!GetSemanticType(types, selected_type)) {
    return false;
  }
  bool ok = false;
  *label = QInputDialog::getText(
      display,
      tr("Semantic Label"),
      tr("Label:"),
      QLineEdit::Normal,
      tr((*label).c_str()),
      &ok).toStdString();
  return ok;
}

bool VectorDisplayThread::GetNavEdgeParams(
    float* width,
    float* max_speed,
    bool* has_door) {
  bool ok = false;
  *width = QInputDialog::getDouble(
      display,
      tr("Set Edge Width"),
      tr("Edge Width:"),
      *width,
      0,
      100,
      2,
      &ok);
  if (!ok) return false;
  *max_speed = QInputDialog::getDouble(
      display,
      tr("Set Max Speed"),
      tr("Max Speed:"),
      *max_speed,
      0,
      2.0,
      2,
      &ok);
  if (!ok) return false;
  QStringList bool_types;
  bool_types << tr("True");
  bool_types << tr("False");
  const string has_door_str = QInputDialog::getItem(
      display,
      tr("Has Door"),
      tr("Has Door:"),
      bool_types,
      ((*has_door) ? 0 : 1),
      false,
      &ok).toStdString();
  *has_door = (has_door_str == "True");
  return ok;
}

void VectorDisplayThread::ChangeMap() {
  QStringList maps;
  {
    ScopedFile fid (mapsFolder + "/atlas.txt", "r");
    if (fid() == NULL) {
      fprintf(stderr, "Error: Unable to load atlas!\n");
      return;
    }
    vector<char> map_entry(4096, 0);
    int map_index = 0;
    while (fscanf(fid, "%d %s", &map_index, map_entry.data()) == 2) {
      maps << tr(map_entry.data());
      map_entry = vector<char>(4096, 0);
    }
  }
  bool ok = false;
  QString map_name = QInputDialog::getItem(
      display, tr("Load Map"), tr("Map:"), maps, 0, false, &ok);
  if (ok && !map_name.isEmpty()) {
    /*
    if (autoUpdateMap && map_name.toStdString().compare(vectorMap.mapName)!= 0) {
      QMessageBox confirmBox;
      confirmBox.setWindowTitle("Confirm");
      confirmBox.setText("Localization auto-update is on.");
      confirmBox.setInformativeText("Do you want to turn off auto-update and change maps?");
      confirmBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
      confirmBox.setDefaultButton(QMessageBox::Ok);
      int ret = confirmBox.exec();

      if (ret == QMessageBox::Ok) {
        autoUpdateMap = false;
      } else {
        return;
      }
    }
    */
    /*
    if (mapEditMode) {
      if (vectorMap.saveMap(map_name_)) {
        printf("Saved map %s\n", map_name_.c_str());
      } else {
        printf("Error saving map %s\n", map_name_.c_str());
        return;
      }
    }
    
    if (navMapMode) {
      if (navMap.SaveMap(map_name_)) {
        printf("Saved navigation map %s\n", map_name_.c_str());
      } else {
        printf("Error saving navigation map %s\n", map_name_.c_str());
        return;
      }
      navMap.LoadMap(map_name.toStdString());
    }
    if (semanticMapMode) {
      if (navMap.SaveSemanticMap(map_name_)) {
        printf("Saved semantic map %s\n", map_name_.c_str());
      } else {
        printf("Error saving semantic map %s\n", map_name_.c_str());
        return;
      }
      navMap.LoadSemanticMap(map_name.toStdString());
    }
    if (semanticViewMode) {
      navMap.LoadSemanticMap(map_name.toStdString());
    }
    if (navViewMode) {
      navMap.LoadMap(map_name.toStdString());
    }*/
    printf("Change map to %s\n", map_name.toStdString().c_str());
    //vectorMap.loadMap(map_name.toStdString(), false);
    map_name_ = map_name.toStdString();
  }
}

/*
void VectorDisplayThread::AutoLocalize()
{
  CobotLocalizationSrv srv;
  srv.request.map = vectorMap.mapName;
  srv.request.angle = robotAngle;
  srv.request.x = robotLoc.x();
  srv.request.y = robotLoc.y();
  autoLocalizeClient.call(srv);
}*/

void VectorDisplayThread::KeyboardEventCallback(
    uint32_t key_code,uint32_t modifiers) {
  switch (key_code) {
    case Qt::Key_C : {
      clearDisplay = true;
    } break;

    case Qt::Key_M : {
      ChangeMap();
      //QInputDialog::
      // blankDisplay = !blankDisplay;
    } break;

    //case Qt::Key_L : {
      //AutoLocalize();
      //printf("AutoLocalizing robot.\n");
    //} break;

    case Qt::Key_U : {
      autoUpdateMap = !autoUpdateMap;
      printf("AutoUpdateMap: %d\n", autoUpdateMap);
    } break;

    //case Qt::Key_N : {
    //  printf("Number of lines in map %s: %i\n", vectorMap.mapName.c_str(),
    //         int(vectorMap.Lines().size()));
    //} break;
  }
  compileDisplay();
}

/*
void VectorDisplayThread::editMap(
    const Vector2f& mouse_down, const Vector2f& mouse_up, float orientation,
    uint32_t modifiers) {
  const vector2f p0(mouse_down.x(), mouse_down.y());
  const vector2f p1(mouse_up.x(), mouse_up.y());

  std::cout << "MODIFIERS: " << modifiers << std::endl;

  switch (modifiers) {
    
    case 0x04: {
      // Add Line
      vector<line2f> lines = vectorMap.Lines();
      lines.push_back(line2f(p0, p1));
      vectorMap.updateMap(lines);
      compileDisplay();
    } break;
    
    case 0x02: {
      // Delete Line
      static const float kMaxError = 0.5;
      vector<line2f> lines = vectorMap.Lines();
      int best_match = -1;
      float best_match_error = FLT_MAX;
      for (size_t i = 0; i < lines.size(); ++i) {
        const line2f& line = lines[i];
        const float match_error = line.closestDistFromLine(p0, true);
        if ((line.P0() - p0).dot(line.P1() - p0) < 0.0f &&
            match_error < best_match_error &&
            match_error < kMaxError) {
          best_match = i;
          best_match_error = match_error;
        }
      }
      if (best_match > -1) {
        lines.erase(lines.begin() + best_match);
        vectorMap.updateMap(lines);
        compileDisplay();
      }
    } break;
  }
}
*/

/*
void VectorDisplayThread::editGraph(
    const Vector2f& mouse_down, const Vector2f& mouse_up,
    float orientation, uint32_t modifiers) {
  static const bool debug = false;

  const vector2f p0(mouse_down.x(), mouse_down.y());
  const vector2f p1(mouse_up.x(), mouse_up.y());
  const float angle = (p1 - p0).angle();
  static const float kMaxError = 0.1;

  const int nearest_vertex_down = navMap.GetClosestVertex(p0, kMaxError);
  const int nearest_vertex_up = navMap.GetClosestVertex(p1, kMaxError);
  const int nearest_edge = navMap.GetClosestEdge(p0, kMaxError);
  const bool down_on_vertex = (nearest_vertex_down != -1);
  const bool up_on_vertex = (nearest_vertex_up != -1);
  const bool click = ((p0 - p1).length() < kMaxError);

  vector<string> semantic_vertex_types;
  semantic_vertex_types.push_back("Office");
  semantic_vertex_types.push_back("Other");
  semantic_vertex_types.push_back("Stair");
  semantic_vertex_types.push_back("Bathroom");
  semantic_vertex_types.push_back("Elevator");
  semantic_vertex_types.push_back("Kitchen");
  semantic_vertex_types.push_back("Printer");
  semantic_vertex_types.push_back("MapExit");

  vector<string> semantic_edge_types;
  semantic_edge_types.push_back("Hallway");
  semantic_edge_types.push_back("Vertical");
  semantic_edge_types.push_back("MapExit");
  const bool dragged_between_vertices =
      down_on_vertex && up_on_vertex &&
      nearest_vertex_down != nearest_vertex_up;

  switch(modifiers) {
    case 0x04: { // Shift
      // Add Edge or Vertex
      if (!dragged_between_vertices && !down_on_vertex) {
        if (navMapMode) {
          navMap.AddVertex(navMap.GetNextVertexIndex(), p0.x, p0.y);
        } else if (semanticMapMode) {
          string roomType;
          string roomLabel;
          if (GetSemanticTypeAndLabel(semantic_vertex_types, &roomType, &roomLabel)) {
            navMap.AddVertex(navMap.GetNextVertexIndex(), p0.x, p0.y, angle,
                             roomType, roomLabel);
          }
        }
      } else if (dragged_between_vertices) {
        // add edge if drag from one vertex to another
        // using default values for width, max_speed, and has_door
        if (navMapMode) {
          float width = 1;
          float max_speed = 1;
          bool has_door = false;
          if (GetNavEdgeParams(&width, &max_speed, &has_door)) {
            if (debug) {
              printf("Edge %d width:%f speed:%f door:%d\n",
                    nearest_edge,
                    width,
                    max_speed,
                    has_door);
            }
            navMap.AddEdge(navMap.Vertices[nearest_vertex_down].handle,
                           navMap.Vertices[nearest_vertex_up].handle,
                           width, max_speed, has_door);
          }
        } else if (semanticMapMode) {
          string edgeType;
          if (GetSemanticType(semantic_edge_types, &edgeType)) {
            navMap.AddEdge(navMap.Vertices[nearest_vertex_down].handle,
                           navMap.Vertices[nearest_vertex_up].handle,
                           1, 1, false, edgeType);
          }
        }
      } else {
        return;
      }
      compileDisplay();
    } break;

    case 0x02: { // Control
      // Delete Edge or Vertex
      if (click && down_on_vertex) {
        navMap.DeleteVertex(navMap.Vertices[nearest_vertex_down].handle);
      } else if (click && nearest_edge > -1) {
        navMap.DeleteEdge(navMap.Vertices[navMap.Edges[nearest_edge].v1].handle,
                          navMap.Vertices[navMap.Edges[nearest_edge].v2].handle);
      } else {
        return;
      }
      compileDisplay();
    } break;

    case 0x01: { // Alt
      // Move edge or vertex
      if (down_on_vertex) {
        navMap.Vertices[nearest_vertex_down].loc = p1;
      } else if (nearest_edge != -1) {
        vector2f shift = p1 - p0;
        int v1 = navMap.Edges[nearest_edge].v1;
        int v2 = navMap.Edges[nearest_edge].v2;
        navMap.Vertices[v1].loc = navMap.Vertices[v1].loc + shift;
        navMap.Vertices[v2].loc = navMap.Vertices[v2].loc + shift;
      } else {
        return;
      }
      compileDisplay();
    } break;

    case 0x03: { // Ctrl-Alt
      // Edit parameters of edge or vertex
      if (down_on_vertex && semanticMapMode) {
        string roomType = navMap.Vertices[nearest_vertex_down].type;
        string roomLabel = navMap.Vertices[nearest_vertex_down].name;
        if (GetSemanticTypeAndLabel(semantic_vertex_types, &roomType, &roomLabel)) {
          navMap.Vertices[nearest_vertex_down].type = roomType;
          navMap.Vertices[nearest_vertex_down].name = roomLabel;
          navMap.Vertices[nearest_vertex_down].theta = angle;
        }
      } else if (!down_on_vertex && nearest_edge != -1) {
        if (navMapMode) {
          float width(navMap.Edges[nearest_edge].width);
          float max_speed(navMap.Edges[nearest_edge].maxSpeed);
          bool has_door(navMap.Edges[nearest_edge].doorWay);
          if (GetNavEdgeParams(&width, &max_speed, &has_door)) {
            if (debug) {
              printf("Edge %d width:%f speed:%f door:%d\n",
                     nearest_edge,
                     width,
                     max_speed,
                     has_door);
            }
            navMap.Edges[nearest_edge].width = width;
            navMap.Edges[nearest_edge].maxSpeed = max_speed;
            navMap.Edges[nearest_edge].doorWay = has_door;
          }
        } else if (semanticMapMode) {
          string edgeType = navMap.Edges[nearest_edge].type;
          if (GetSemanticType(semantic_edge_types, &edgeType)) {
            navMap.Edges[nearest_edge].type = edgeType;
          }
        }
      }
      compileDisplay();
    } break;
  }
}
*/
void VectorDisplayThread::MouseEventCallback(
    const Vector2f& mouse_down,
    const Vector2f& mouse_up, float orientation,
    uint32_t modifiers) {
  static const bool debug = false;
  if (saveLocs && modifiers != 0) {
    FILE* fid = (*saveLocsFile)();
    if (saveOrientations) {
      const float angle = atan2(
          (mouse_up-mouse_down).y(), (mouse_up-mouse_down).x());
      fprintf(fid, "%f, %f, %f\n",
              mouse_down.x(), mouse_down.y(), angle);
      printf("Saved mouse_down: %f, %f, %f\n",
             mouse_down.x(), mouse_down.y(), angle);
    } else {
      fprintf(fid, "%f, %f, %f, %f\n",
              mouse_down.x(), mouse_down.y(), mouse_up.x(), mouse_up.y());
      printf("Saved mouse_location: %f, %f, %f, %f\n",
             mouse_down.x(), mouse_down.y(), mouse_up.x(), mouse_up.y());
    }
    return;
  }
  /*
  if (mapEditMode) {
    editMap(mouse_down, mouse_up, orientation, modifiers);
    if (modifiers == 0x01) {
      printf("Length: %f\n", (mouse_down - mouse_up).norm());
    }
    return;
  }
*/
  //if (navMapMode || semanticMapMode) {
  //  editGraph(mouse_down, mouse_up, orientation, modifiers);
  //  return;
  //}
  {
    CobotRemoteInterfaceSrv srv;
    srv.request.loc_x = mouse_down.x();
    srv.request.loc_y = mouse_down.y();
    srv.request.orientation = orientation;
    srv.request.command_num = 0;
    //srv.request.map = vectorMap.mapName;
    srv.request.map = "map_name";
    srv.request.distance_tolerance = 0.25;
    srv.request.angle_tolerance = RAD(5.0);
    switch (modifiers) {
      case 0x04: {
        // Set Position
        if (debug) {
          printf("SetPosition: %7.3f, %7.3f %6.1f\u00b0\n",
              mouse_down.x(), mouse_down.y(), DEG(orientation));
        }
        srv.request.command_type = 0x0002;
        client = localizationClient;
      } break;
      case 0x02: {
        // Set Target
        if (debug) {
          printf("SetTarget: %7.3f, %7.3f %6.1f\u00b0\n",
              mouse_down.x(), mouse_down.y(), DEG(orientation));
        }
        srv.request.command_type = 0x0034;
        client = managerClient;
      } break;
    }
    if (modifiers == 0x04 || modifiers == 0x02) {
      if (client.call(srv)) {
      } else {
        printf("Failed to send command!\n");
      }
    }
  }
  if (modifiers == 0x04) {  // Set Position
    localizationInitMsg.header.seq++;
    localizationInitMsg.header.stamp = ros::Time::now();
    localizationInitMsg.header.frame_id = "map";
    localizationInitMsg.pose.pose.position.x = mouse_down.x();
    localizationInitMsg.pose.pose.position.y = mouse_down.y();
    localizationInitMsg.pose.pose.position.z = 0;
    localizationInitMsg.pose.pose.orientation.w = cos(orientation*0.5);
    localizationInitMsg.pose.pose.orientation.z = sin(orientation*0.5);
    localizationInitMsg.pose.pose.orientation.x = 0;
    localizationInitMsg.pose.pose.orientation.y = 0;
    static const float mouse_downUncertainty = sq(0.01);
    static const float angleUncertainty = sq(RAD(1.0));
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        int ind = i*6+j;
        localizationInitMsg.pose.covariance[ind] = 0;
      }
    }
    localizationInitMsg.pose.covariance[0] =
        localizationInitMsg.pose.covariance[7] = mouse_downUncertainty;
    localizationInitMsg.pose.covariance[35] = angleUncertainty;

    initialPosePublisher.publish(localizationInitMsg);
  }
}

void VectorDisplayThread::Zoom(float zoom) {
  if (display) display->Zoom(zoom);
}

/*

void VectorDisplayThread::drawMap(
    vector<VectorDisplay::Line>* lines,
    vector<VectorDisplay::Color>* lineColors) {
  for (size_t i = 0; i < vectorMap.Lines().size(); i++) {
    lines->push_back(VectorDisplay::Line(
        Vector2f(vectorMap.Line(i).P0().x, vectorMap.Line(i).P0().y),
        Vector2f(vectorMap.Line(i).P1().x, vectorMap.Line(i).P1().y)));
    lineColors->push_back(VectorDisplay::Color(0.32, 0.49, 0.91, 1.0));
  }
}

*/

// void VectorDisplayThread::humanDetectionCallback(
//     const CobotHumansDetected& msg) {
//   humansMsg = msg;
//   tHumanDetect = GetTimeSec();
//   compileDisplay();
// }

// void VectorDisplayThread::humanTrackingCallback(const vector_slam_msgs::CobotHumansClassified& msg)
// {
//   classifiedHumansMsg = msg;
//   tHumanTrack = GetTimeSec();
//   compileDisplay();
// }

//void VectorDisplayThread::cobotLocalizationCallback(const CobotLocalizationMsg& msg) {
 // robotLoc = Vector2f(msg.x, msg.y);
  //robotAngle = msg.angle;

//  if (autoUpdateMap && msg.map.compare(vectorMap.mapName)!= 0) {
 //   if (mapEditMode) {
/*
      if (vectorMap.saveMap(map_name_)) {
        printf("Saved map %s\n", map_name_.c_str());
      } else {
        printf("Error saving map %s\n", map_name_.c_str());
        return;
      }
      */
  //  }
/*
    if (navMapMode) {
      if (navMap.SaveMap(map_name_)) {
        printf("Saved navigation map %s\n", map_name_.c_str());
      } else {
        printf("Error saving navigation map %s\n", map_name_.c_str());
        return;
      }
      navMap.LoadMap(msg.map.c_str());
    }
    if (semanticMapMode) {
      if (navMap.SaveSemanticMap(map_name_)) {
        printf("Saved semantic map %s\n", map_name_.c_str());
      } else {
        printf("Error saving semantic map %s\n", map_name_.c_str());
        return;
      }
      navMap.LoadSemanticMap(msg.map.c_str());
    }
    if (semanticViewMode) {
      navMap.LoadSemanticMap(msg.map.c_str());
    }
    if (navViewMode) {
      navMap.LoadMap(msg.map.c_str());
    }*/

    //vectorMap.loadMap(msg.map.c_str(), false);
//    map_name_ = msg.map;
//  }
//  compileDisplay();
//}

void VectorDisplayThread::cobotAnomalyCallback(const CobotAnomalyMonitorRectangleMsg& msg) {
  static const int X = 0;
  static const int Y = 1;
  static const float BIG_NUM = 1000000.0;
  anomalyLLX = msg.lowerLeft[X];
  anomalyLLY = msg.lowerLeft[Y];
  anomalyURX = msg.upperRight[X];
  anomalyURY = msg.upperRight[Y];
  anomaly = msg.anomaly;

  if (anomalyLLX < -BIG_NUM) anomalyLLX = -BIG_NUM;
  if (anomalyLLY < -BIG_NUM) anomalyLLY = -BIG_NUM;
  if (anomalyURX > BIG_NUM) anomalyURX = BIG_NUM;
  if (anomalyURY > BIG_NUM) anomalyURY = BIG_NUM;
}

void VectorDisplayThread::kinectScanCallback(
    const sensor_msgs::LaserScan& msg) {
  kinectScanMsg = msg;
  compileDisplay();
}

void VectorDisplayThread::laserCallback(const sensor_msgs::LaserScan& msg) {
  laserScanMsg = msg;
  tLaser = GetTimeSec();
  compileDisplay();
}

void VectorDisplayThread::filteredPointCloudCallback(
    const sensor_msgs::PointCloud& msg) {
  pointCloudMsg = msg;
  tPointCloud = GetTimeSec();
  compileDisplay();
}


void VectorDisplayThread::cobotStatusCallback(const CobotStatusMsg& msg) {
  pathPlan.resize(msg.pathPlan_x.size());
  for (unsigned int i = 0; i < msg.pathPlan_x.size(); i++) {
    pathPlan[i] = Vector2f(msg.pathPlan_x[i], msg.pathPlan_y[i]);
  }
  tPathPlan = GetTimeSec();
  compileDisplay();
}

void VectorDisplayThread::statusCallback(
    const ros::MessageEvent<LidarDisplayMsg const>& msgEvent) {
  static const bool debug = false;
  const vector_slam_msgs::LidarDisplayMsgConstPtr &msg = msgEvent.getConstMessage();
  bool duplicate = false;
  unsigned int i = 0;
  if (debug) {
    printf("Received message from %s\n",
           msgEvent.getPublisherName().c_str());
  }
  for (; i < displayProviders.size() && !duplicate; i++) {
    if (displayProviders[i].compare(msgEvent.getPublisherName())== 0)
      duplicate = true;
  }
  if (debug) printf("Duplicate:%d, i:%d\n", duplicate, i);
  if (duplicate) {
    i--;
    displayMsgs[i] = *msg;
  } else {
    displayMsgs.push_back(*msg);
    displayProviders.push_back(msgEvent.getPublisherName());
  }
  compileDisplay();
}

void VectorDisplayThread::clearDisplayMessages() {
  anomaly = 0.0;
  displayMsgs.clear();
  displayProviders.clear();
  laserScanMsg.ranges.clear();
  kinectScanMsg.ranges.clear();
  //humansMsg.humans.clear();
  //classifiedHumansMsg.classified_humans.clear();
  pathPlan.clear();
}

void VectorDisplayThread::compileDisplay() {
  static double tLast = 0.0;
  static const double MessageTimeout = 1.0;
  static const VectorDisplay::Color LidarPointColor(0xFFF0761F);
  static const VectorDisplay::Color KinectScanColor(0xFFFF0505);
  static const VectorDisplay::Color PointCloudColor(0xFFDE2352);
  static const bool debug = false;
  static const float scaleExp = 8.0;
  if (viewVectorFile) return;

  if (debug) printf("GUI updated!\n");
  if (GetTimeSec()-tLast< 1.0 / maxFps) return;
  tLast = GetTimeSec();

  lines.clear();
  points.clear();
  circles.clear();
  quads.clear();
  circleColors.clear();
  lineColors.clear();
  pointColors.clear();
  quadColors.clear();
  textColors.clear();
  textStrings.clear();
  textLocs.clear();
  textHeights.clear();
  textInWindowCoords.clear();

  if (!blankDisplay) {
    //drawMap(&lines, &lineColors);
  }
  /*
  if (navMapMode || semanticMapMode || navViewMode) {
    VectorDisplay::Color roomLabel(0.0, 0.0, 0.0, 1.0);
    const vector<Vertex>& vertices = navMap.Vertices;
    for(unsigned int i=0; i<navMap.numVertices; ++i) {
      vector2f v = navMap.Vertices[i].loc;
      points.push_back(Vector2f(V2COMP(v)));
      pointColors.push_back(VectorDisplay::Color(0xFF008800));
      if (semanticMapMode) {
        textStrings.push_back(vertices[i].name);
        textLocs.push_back(Vector2f(V2COMP(v)));
        textColors.push_back(roomLabel);
        textHeights.push_back(0.5);
        const Vector2f p0(V2COMP(vertices[i].loc));
        vector2f heading;
        heading.heading(vertices[i].theta);
        const Vector2f p1(V2COMP(vertices[i].loc + 0.5*heading));
        lines.push_back(VectorDisplay::Line(p0, p1));
        lineColors.push_back(VectorDisplay::Color(0x7F404040));
      }
    }
    for(unsigned int i=0; i<navMap.numEdges; i++) {
      int v1 = navMap.Edges[i].v1;
      int v2 = navMap.Edges[i].v2;

      lines.push_back(VectorDisplay::Line(Vector2f(V2COMP(vertices[v1].loc)),
                                          Vector2f(V2COMP(vertices[v2].loc))));
      lineColors.push_back(VectorDisplay::Color(0xFFFF00FF));
    }
  }*/
/*
  if (semanticViewMode) {
    // TODO -- fix logic so we don't need this repeated code?
    VectorDisplay::Color roomLabel(0.0, 0.0, 0.0, 1.0);
    const vector<Vertex>& vertices = navMap.Vertices;
    for(unsigned int i=0; i<navMap.numVertices; ++i) {
      vector2f v = navMap.Vertices[i].loc;
      points.push_back(Vector2f(V2COMP(v)));
      pointColors.push_back(VectorDisplay::Color(0xFF008800));

      textStrings.push_back(vertices[i].name);
      textLocs.push_back(Vector2f(V2COMP(v)));
      textColors.push_back(roomLabel);
      textHeights.push_back(0.5);

      const Vector2f p0(V2COMP(vertices[i].loc));
      vector2f heading;
      heading.heading(vertices[i].theta);
      const Vector2f p1(V2COMP(vertices[i].loc + 0.5*heading));
      lines.push_back(VectorDisplay::Line(p0, p1));
      lineColors.push_back(VectorDisplay::Color(0x7F404040));
    }
  }*/

  if (showAnomalyProb && (anomaly != 0.0)) {
    printf("ll: (%f,%f), ur: (%f,%f), anomaly: %f\n",anomalyLLX,anomalyLLY,
        anomalyURX,anomalyURY,anomaly);
    VectorDisplay::Quad q = VectorDisplay::Quad(
        Vector2f(anomalyLLX, anomalyLLY), Vector2f(anomalyURX, anomalyLLY),
        Vector2f(anomalyURX, anomalyURY), Vector2f(anomalyLLX, anomalyURY));
    quads.push_back(q);
    quadColors.push_back(VectorDisplay::Color(0.98, 0.98 * (1.0 -
        pow(anomaly, scaleExp)), 0.98 * (1 - pow(anomaly, scaleExp)), 1.0));
    if (debug) {
      printf("lower: (%f, %f) upper: (%f, %f) anomaly: %f\n",
          anomalyLLX, anomalyLLY, anomalyURX, anomalyURY, anomaly);
    }
  }

  // Draw humans detected.
  /*
  if (GetTimeSec()-tHumanDetect < MessageTimeout) {
    for (size_t i = 0; i < humansMsg.humans.size(); ++i) {
      const Vector2f human_local(
          humansMsg.humans[i].pose.position.x, humansMsg.humans[i].pose.position.y);
      const Vector2f human_world =
          robotLoc + Rotation2Df(robotAngle) * human_local;
      float human_orient = 2 * acos(humansMsg.humans[i].pose.orientation.w); // doesn't properly handle non-detections
      const Vector2f orientEnd = human_world +
          (Rotation2Df(robotAngle + human_orient) * Vector2f(1.0, 0) * 0.35);
      static const VectorDisplay::Color kHumansColor(0.0, 0.0, 0.0, 1.0);
      textLocs.push_back(human_world + Vector2f(-0.4, 0.25) * 0.25);
      textHeights.push_back(0.25);
      textColors.push_back(kHumansColor);
      textStrings.push_back("H");
      lines.push_back(VectorDisplay::Line(human_world, orientEnd));
      lineColors.push_back(kHumansColor);
   }
  }
  */
  
  // Draw humans classified.
  /*
  static const float kHumanBoxSize = 0.35;
  if (GetTimeSec()-tHumanTrack < MessageTimeout) {
    for (size_t i = 0; i < classifiedHumansMsg.classified_humans.size(); ++i) {
      const Vector2f human_local(
          classifiedHumansMsg.classified_humans[i].position.x,
          classifiedHumansMsg.classified_humans[i].position.y);
      const Vector2f human_world =
          robotLoc + (Rotation2Df(robotAngle) * human_local);
      float angle_local = classifiedHumansMsg.classified_humans[i].orientation;
      float angle_world = robotAngle + angle_local; // doesn't properly handle non-detections
      Rotation2Df human_world_angle = Rotation2Df(angle_world);
      const Vector2f h0 = human_world + human_world_angle * Vector2f(-0.5, -0.5) * kHumanBoxSize;
      const Vector2f h1 = human_world + human_world_angle * Vector2f(0.5, -0.5) * kHumanBoxSize;
      const Vector2f h2 = human_world + human_world_angle * Vector2f(0.5, 0.5) * kHumanBoxSize;
      const Vector2f h3 = human_world + human_world_angle * Vector2f(-0.5, 0.5) * kHumanBoxSize;
      const Vector2f orientEnd = human_world + (human_world_angle * Vector2f(1.0, 0) * kHumanBoxSize);

      uint hash = classifiedHumansMsg.classified_humans[i].id * 2654435761;
      VectorDisplay::Color color(hash);
      color.a = classifiedHumansMsg.classified_humans[i].confidence;

      lines.push_back(VectorDisplay::Line(h0, h1));
      lines.push_back(VectorDisplay::Line(h1, h2));
      lines.push_back(VectorDisplay::Line(h2, h3));
      lines.push_back(VectorDisplay::Line(h3, h0));
      lines.push_back(VectorDisplay::Line(human_world, orientEnd));
      lineColors.push_back(color);
      lineColors.push_back(color);
      lineColors.push_back(color);
      lineColors.push_back(color);
      lineColors.push_back(color);
    }
  }*/

  for (unsigned int j = 0; j < displayMsgs.size(); j++) {
    const LidarDisplayMsg& displayMsg = displayMsgs[j];
    unsigned int numLines =
        min(min(displayMsg.lines_p1x.size(),
            displayMsg.lines_p1y.size()),
            min(displayMsg.lines_p2x.size(),
                displayMsg.lines_p2y.size()));
    for (unsigned int i = 0; i < numLines; i++) {
      lines.push_back(VectorDisplay::Line(
          Vector2f(displayMsg.lines_p1x[i], displayMsg.lines_p1y[i]),
          Vector2f(displayMsg.lines_p2x[i], displayMsg.lines_p2y[i])));
      if (i < displayMsg.lines_col.size()) {
        lineColors.push_back(VectorDisplay::Color(displayMsg.lines_col[i]));
      }
    }
    unsigned int numPoints = min(displayMsg.points_x.size(),
        displayMsg.points_y.size());
    for (unsigned int i = 0; i < numPoints; i++) {
      points.push_back(
          Vector2f(displayMsg.points_x[i], displayMsg.points_y[i]));
      if (i < displayMsg.points_col.size())
        pointColors.push_back(
            VectorDisplay::Color(displayMsg.points_col[i]));
    }
    unsigned int numCircles = min(displayMsg.circles_x.size(),
        displayMsg.circles_y.size());
    for (unsigned int i = 0; i < numCircles; i++) {
      circles.push_back(
          Vector2f(displayMsg.circles_x[i], displayMsg.circles_y[i]));
      if (i < displayMsg.circles_col.size())
        circleColors.push_back(
            VectorDisplay::Color(displayMsg.circles_col[i]));
    }
    for (size_t i = 0; i < displayMsg.text.size(); ++i) {
      textLocs.push_back(Vector2f(displayMsg.text_x[i], displayMsg.text_y[i]));
    }
    for (size_t i = 0; i < displayMsg.text_col.size(); ++i) {
      textColors.push_back(VectorDisplay::Color(displayMsg.text_col[i]));
    }
    textStrings.insert(textStrings.end(), displayMsg.text.begin(),
                       displayMsg.text.end());
    textHeights.insert(textHeights.end(), displayMsg.text_height.begin(),
                       displayMsg.text_height.end());
    textInWindowCoords.insert(textInWindowCoords.end(),
                              displayMsg.text_in_window_coords.begin(),
                              displayMsg.text_in_window_coords.end());
  }
  if (debug) {
    printf("lines: %d points: %d circles: %d\n",
        static_cast<int>(lines.size()), static_cast<int>(points.size()),
        static_cast<int>(circles.size()));
  }
  if ((GetTimeSec()-tPointCloud < MessageTimeout || persistentDisplay)
      && liveView) {
    const Rotation2Df robot_angle(robotAngle);
    for (size_t i = 0; i < pointCloudMsg.points.size(); ++i) {
      static const float kMaxNormalAngle = RAD(30.0);
      static const float kMaxNormalCosine = cos(kMaxNormalAngle);
      static const size_t kZNormalIndex = 2;
      if (fabs(pointCloudMsg.channels[kZNormalIndex].values[i]) >
          kMaxNormalCosine) {
        continue;
      }
      const Vector2f point =
          robot_angle * Vector2f(V2COMP(pointCloudMsg.points[i])) + robotLoc;
      points.push_back(point);
      pointColors.push_back(PointCloudColor);
    }
  }
  if (liveView) {
    size_t i = 0;
    float a = 0.0;
    Vector2f p(0.0, 0.0);
    for (i = 0, a = robotAngle + kinectScanMsg.angle_min;
        i < kinectScanMsg.ranges.size();
        i++, a += kinectScanMsg.angle_increment) {
      if (kinectScanMsg.ranges[i]<= kinectScanMsg.range_min ||
          kinectScanMsg.ranges[i]>= kinectScanMsg.range_max) {
        continue;
      }
      p = Rotation2Df(a) * Vector2f(1.0, 0.0) * kinectScanMsg.ranges[i];
      points.push_back(robotLoc + p);
      pointColors.push_back(KinectScanColor);
    }
  }
  if ((GetTimeSec()-tLaser < MessageTimeout || persistentDisplay)
      && liveView) {
    unsigned int i = 0;
    float a = 0.0;
    const Vector2f laserLoc =
        Rotation2Df(robotAngle) * Vector2f(0.145, 0.0) + robotLoc;
    for (i = 0, a = robotAngle + laserScanMsg.angle_min;
        i < laserScanMsg.ranges.size();
        i++, a+= laserScanMsg.angle_increment) {
      if (laserScanMsg.ranges[i]<= laserScanMsg.range_min ||
          laserScanMsg.ranges[i]>= laserScanMsg.range_max) {
        continue;
      }
      const Vector2f p =
          Rotation2Df(a) * Vector2f(1.0, 0.0) * laserScanMsg.ranges[i] +
          laserLoc;
      points.push_back(p);
      pointColors.push_back(LidarPointColor);
    }
  }

  if (GetTimeSec()-tPathPlan < MessageTimeout || persistentDisplay) {
    if (pathPlan.size()>0) {
      lines.push_back(VectorDisplay::Line(pathPlan[0], robotLoc));
      lineColors.push_back(VectorDisplay::Color(0xFF00FF00));
      points.push_back(pathPlan[0]);
      pointColors.push_back(VectorDisplay::Color(0xFFFF0000));
    }
    for (int i = 0; i < static_cast<int>(pathPlan.size()) - 1; i++) {
      lines.push_back(VectorDisplay::Line(pathPlan[i], pathPlan[i+1]));
      lineColors.push_back(VectorDisplay::Color(0xFF00FF00));
      points.push_back(pathPlan[i]);
      points.push_back(pathPlan[i+1]);
      pointColors.push_back(VectorDisplay::Color(0xFFFF0000));
      pointColors.push_back(VectorDisplay::Color(0xFFFF0000));
    }
  }

  display->updateDisplay(
      robotLoc, robotAngle, 100.0, lines, points, circles, quads,
      lineColors, pointColors, circleColors, quadColors, textLocs, textStrings,
      textHeights, textColors, textInWindowCoords);
}

void VectorDisplayThread::run() {
  static const bool kTextOnly = false;
  static const bool debug = false;
  if (testMode) {
    vector<bool> text_in_window_coords;
    int char_offset = 0;
    string str(
      "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Donec au "
      "metus risus, in scelerisque lorem pharetra eget. Aliquam facilisi\n"
      "luctus mauris condimentum ornare. Morbi lacinia enim ac dictum tpo "
      "\t\tSuspendisse potenti. Maecs aliquet urna dolor, non commodo aue\n"
      "\tconsequat sed. Vestibulum mlis mollis sem, id convallis leo ulic "
      "in. Sed vel odio placerat urna dictum rutrum.");
    while (runApp && ros::ok()) {
      textLocs.clear();
      textStrings.clear();
      textHeights.clear();
      textColors.clear();
      static const int kNumTextParagraphs = 20;
      for (int i = 0; i < kNumTextParagraphs; ++i) {
        textLocs.push_back(Vector2f(10.0, 4.0 * static_cast<float>(i)));
        for (size_t j = 0; j < str.length(); ++j) {
          if (str[j] >= 'a' && str[j] <= 'z') {
            str[j] = 'a' + ((str[j] - 'a' + char_offset) % 26);
          }
        }
        textStrings.push_back(str);
        textHeights.push_back(1.0);
        textColors.push_back(VectorDisplay::Color(0xFF000000));
        char_offset = (char_offset + 1) % 26;
      }
      textLocs.push_back(Vector2f(0.0, 10.0));
      textStrings.push_back(StringPrintf("%.3f", GetTimeSec()));
      textHeights.push_back(1.0);
      textColors.push_back(VectorDisplay::Color(0xFF0000F0));
      display->updateText(textLocs,
                          textStrings,
                          textHeights,
                          textColors,
                          text_in_window_coords);
      if (!kTextOnly) {
        static const float scale = 0.005;
        static const int numLines = 10;
        static const int numPoints = 800;
        vector<VectorDisplay::Line> lines;
        vector<Vector2f> points;
        double dTheta = 2.0*M_PI/static_cast<double>(numLines);
        Vector2f offset(1000.0, 0.0);
        double theta = 0.0;
        double omega = RAD(30.0);
        const float angle = angle_mod<double>(omega*GetTimeSec());
        for (int i = 0; i < numLines; i++) {
          const Vector2f p(1000.0, 0.0);
          const Vector2f p1 =
              scale * (Rotation2Df(theta + angle) * p);
          const Vector2f p2 =
              scale * (Rotation2Df(theta + dTheta + angle) * p);
          lines.push_back(VectorDisplay::Line(p1, p2));
          theta += dTheta;
        }
        dTheta = 2.0*M_PI/static_cast<double>(numPoints);
        theta = 0.0;
        for (int i = 0; i < numPoints; i++) {
          Vector2f p(3500.0, 0.0);
          p = p*max(0.0, 1.1+sin(sin(2.0*theta)*M_PI))/2.0;
          p = Rotation2Df(theta + angle) * p + offset;
          p *= scale;
          points.push_back(p);
          theta += dTheta;
        }
        display->updateLines(lines, vector<VectorDisplay::Color>());
        display->updatePoints(points, vector<VectorDisplay::Color>());
      }
      Sleep(0.016);
    }
    if (debug) {
      printf("Terminating testMode thread. runApp:%d ok():%d\n",
          runApp?1:0, ros::ok()?1:0);
    }
  } else {
    //vectorMap = VectorMap(map_name_, mapsFolder.c_str(), false);

    ros::Subscriber guiSub;
    ros::Subscriber statusSub;
    ros::Subscriber laserSub;
    ros::Subscriber kinectScanSub;
    ros::Subscriber localizationSub;
    ros::Subscriber anomalySub;
    ros::Subscriber planesSub;
    ros::Subscriber humansSub;
    ros::Subscriber classifiedHumansSub;

    //humansSub = node_handle_->subscribe(
    //    "Cobot/HumanDetect/Humans", 1,
    //    &VectorDisplayThread::humanDetectionCallback, this);
    //classifiedHumansSub = node_handle_->subscribe(
    //    "/Cobot/HumanDetect/ClassifiedHumans", 1,
    //    &VectorDisplayThread::humanTrackingCallback, this);
    statusSub = node_handle_->subscribe(
        "Cobot/Status", 1, &VectorDisplayThread::cobotStatusCallback, this);
    laserSub = node_handle_->subscribe(
        "Cobot/Laser", 1, &VectorDisplayThread::laserCallback, this);
    //localizationSub = node_handle_->subscribe(
    //    "Cobot/Localization", 1,
    //    &VectorDisplayThread::cobotLocalizationCallback, this);
    anomalySub =node_handle_->subscribe(
        "Cobot/ExecutionMonitor/AnomalyMonitorRect",
        1, &VectorDisplayThread::cobotAnomalyCallback, this);
    guiSub = node_handle_->subscribe(
        "VectorSLAM/VectorLocalization/Gui", 1,
        &VectorDisplayThread::statusCallback, this);
    kinectScanSub = node_handle_->subscribe(
        "Cobot/Kinect/Scan", 1,
        &VectorDisplayThread::kinectScanCallback, this);
    // planesSub = node_handle_->subscribe(
    //     "Cobot/Kinect/FilteredPointCloud", 1,
    //     &VectorDisplayThread::filteredPointCloudCallback, this);
    compileDisplay();

    while (runApp && ros::ok()) {
      ros::spinOnce();
      Sleep(0.01);
      if (clearDisplay) {
        clearDisplayMessages();
        clearDisplay = false;
        compileDisplay();
      }
    }
    if (debug) {
      printf("Terminating thread. runApp:%d ok():%d\n",
          runApp?1:0, ros::ok()?1:0);
    }
  }
  if (app != 0)
    app->quit();
}

void VectorDisplayThread::setOptions(
    bool testMode, string mapName, bool saveLocs, bool liveView,
    bool persistentDisplay, bool saveOrientations, bool blankDisplay,
    float maxFps, bool mapEditMode, bool semanticMapMode,
    bool semanticViewMode, bool navViewMode, bool viewVectorFile) {

  this->testMode = testMode;
  map_name_ = mapName;
  this->liveView = liveView;
  this->persistentDisplay = persistentDisplay;
  this->blankDisplay = blankDisplay;
  this->saveLocs = saveLocs;
  this->saveOrientations = saveOrientations;
  this->maxFps = maxFps;
  this->mapEditMode = mapEditMode;
  //this->navMapMode = navMapMode;
  this->semanticMapMode = semanticMapMode;
  this->semanticViewMode = semanticViewMode;
  this->navViewMode = navViewMode;
  this->viewVectorFile = viewVectorFile;
  //if (navMapMode || semanticMapMode || navViewMode) {
  //  display->SetPrimitivesSizes(3.0, 1.5);
  //}
  //if (navMapMode || navViewMode) {
  //  navMap.LoadMap(map_name_);
  //} else if (semanticMapMode || semanticViewMode) {
  //  navMap.LoadSemanticMap(map_name_);
  //}
  if (saveLocs) {
    saveLocsFile = new ScopedFile("savedLocs.txt", "aw");
  }
}

VectorDisplayThread::VectorDisplayThread(
    const std::string& _mapsFolder, VectorDisplay* disp,
    ros::NodeHandle* node_handle, QApplication* qapp, QObject* parent) :
    node_handle_(node_handle), app(qapp), display(disp), saveLocsFile(NULL) {
  autoUpdateMap = true;
  runApp = true;
  mapEditMode = false;
  //navMapMode = false;
  semanticMapMode = false;
  semanticViewMode = false;
  clearDisplay = false;
  showAnomalyProb = false;
  anomaly = 0.0;
  tPointCloud = 0.0;
  persistentDisplay = false;
  liveView = false;
  tLaser = 0.0;
  tPathPlan = 0.0;
  tHumanDetect = 0.0;
  tHumanTrack = 0.0;
  this->setOptions(false, "GHC7", false, false, true, false, false, 60.0,
                   false, false, false, false, false);
  mapsFolder = _mapsFolder;
  pathPlan.clear();
  localizationInitMsg.header.seq = 0;
  initialPosePublisher = node_handle_->advertise<PoseWithCovarianceStamped>(
      string("initialpose"), 5, false);
  localizationClient = node_handle_->serviceClient<CobotRemoteInterfaceSrv>(
      "Cobot/VectorLocalization/RemoteInterface");
  managerClient = node_handle_->serviceClient<CobotRemoteInterfaceSrv>(
      "Cobot/RemoteInterface");
  autoLocalizeClient = node_handle->serviceClient<CobotLocalizationSrv>(
      "Cobot/VectorLocalization/AutoLocalize");
}

VectorDisplayThread::~VectorDisplayThread() {
  if (saveLocsFile != NULL) {
    delete saveLocsFile;
  }
/*
  if (mapEditMode) {
    if (vectorMap.saveMap(map_name_)) {
      printf("Saved map %s\n", map_name_.c_str());
    } else {
      printf("Error saving map %s\n", map_name_.c_str());
    }
  }*/
/*
  if (navMapMode) {
    if (navMap.SaveMap(map_name_)) {
      printf("Saved navigation map %s\n", map_name_.c_str());
    } else {
      printf("Error saving navigation map %s\n", map_name_.c_str());
    }
  }
  if (semanticMapMode) {
    if (navMap.SaveSemanticMap(map_name_)) {
      printf("Saved semantic map %s\n", map_name_.c_str());
    } else {
      printf("Error saving semantic map %s\n", map_name_.c_str());
    }
  }*/
  fflush(stdout);
}
