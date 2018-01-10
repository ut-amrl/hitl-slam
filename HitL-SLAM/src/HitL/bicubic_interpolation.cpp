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
* \file    wifi_learn.cpp
* \brief   WiFi Map building app
* \author  Joydeep Biswas, (C) 2012
*/
//========================================================================

#include <stdio.h>
#include "wifi_map.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <termios.h>

#include "cobot_msgs/CobotAccessPointsMsg.h"
#include <eigen3/Eigen/Dense>
#include "cobot_msgs/LidarDisplayMsg.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "cobot_msgs/CobotOdometryMsg.h"
#include "cobot_msgs/CobotRemoteInterfaceSrv.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "proghelp.h"
#include "popt_pp.h"
#include "configreader.h"
#include "terminal_utils.h"

bool run = true;
int debugLevel = -1;

float s_f = 2.8636;
float l = 17.8;
float s_n = 2.0;
float xMin = -30.5077;
float xMax = 35.0390;
float yMin = -59.3913;
float yMax = 59.2158;
float step = 1.5;
float rssiMean = -100;

using namespace Eigen;
using namespace std;

vector<Vector2f> locs;
vector<uint64_t> bssids;
vector<float> rssiValues;
vector<uint64_t> uniqueBssids;

vector< vector<Vector2f> > apLocs;
vector< VectorXf > apRssiValues;
vector< MatrixXf > apKMatrices;
vector< MatrixXf > apMeans;
vector< MatrixXf > apVariances;
vector< uint32_t > apReadings;

void LoadParameters()
{
  WatchFiles watch_files;
  ConfigReader config(ros::package::getPath("cobot_linux").append("/").c_str());;
  
  config.init(watch_files);
  config.addFile("../robot.cfg");
  config.addFile("config/localization_parameters.cfg");
  if(!config.readFiles()){
    printf("Failed to read config\n");
    exit(1);
  }
  /*
  {
    ConfigReader::SubTree c(config,"initialConditions");
    
    c.getVec2f("loc",initialLoc);
    c.getReal("angle", initialAngle);
    c.getReal("locUncertainty", locUncertainty);
    c.getReal("angleUncertainty", angleUncertainty);
    locUncertainty = max(1.0, locUncertainty);
    angleUncertainty = max(RAD(30.0), angleUncertainty);
  }
  */
}

bool loadData(const char* mapName)
{
  static const bool debug = true;
  char* fileName = (char*) malloc(4096);
  snprintf(fileName, 4095,"%s/../maps/%s/wifiLog.txt", ros::package::getPath("cobot_linux").c_str(),mapName);
  FILE* fid = fopen(fileName, "r");
  if(fid==NULL)
    return false;
  
  float t,x,y,theta, rssi, age;
  long long unsigned int bssid;
  char map[4096];
  
  if(debug) printf("Loading %s\n",fileName);
  
  Vector2f loc;
  while(fscanf(fid, "%f, %f,%f, %f, %s , %llu, %f, %f\n",&t,&x,&y,&theta,map,&bssid,&rssi,&age)==8){
    //if(debug) printf("loc: %8.3f,%8.3f %6.1f\u00b0, %012llX, %6.1f\n",x,y,DEG(theta), bssid, rssi);
    loc.x() = x;
    loc.y() = y;
    locs.push_back(loc);
    bssids.push_back(bssid);
    rssiValues.push_back(rssi-rssiMean);
  }
  
  if(debug) printf("Read %d records\nUnique BSSIDs:\n",int(locs.size()));
  
  for(unsigned int i=0; i<bssids.size(); i++){
    bool found = false;
    for(unsigned int j=0; !found && j<uniqueBssids.size(); j++){
      found = uniqueBssids[j]==bssids[i];
    } 
    if(!found){
      uniqueBssids.push_back(bssids[i]);
    }
  }
  if(debug) printf("%d Unique APs\n",int(uniqueBssids.size()));
  
  return true;
}


inline float K(const Vector2f &x1, const Vector2f &x2)
{
  return s_f*s_f*exp(-1.0/(2.0*l*l)*((x1-x2).squaredNorm()));
}


MatrixXf Kmatrix(const vector<Vector2f> &X){
  int N = X.size();
  MatrixXf Km(N,N), I(N,N);
  I.setIdentity();
  I.setIdentity();
  for(int i=0; i<N; i++){
    for(int j=0; j<=i; j++){
      Km(i,j) = K(X[i],X[j]);
      Km(j,i) = Km(i,j);
    }
  }
  Km = (Km + s_n*s_n*I).inverse();
  return Km;
}

void gp(float x, float y, const vector<Vector2f> &locs, const VectorXf &rssiValues, const MatrixXf &K_mat, float &mean, float &variance)
{
  int N = locs.size();
  MatrixXf Kv(N,1), Kvt;
  Vector2f l(x,y);
  for(int i=0; i<N; i++){
    Kv(i,0) = K(l,locs[i]);
  }
  Kvt = Kv.transpose();
  MatrixXf u = Kvt*K_mat;
  MatrixXf ret;
  ret = u*rssiValues;
  mean = ret(0,0) + rssiMean;
  ret = (u*Kv);
  variance = ret(0,0);
  variance = K(l,l) - variance;
  
}

bool saveMap(const char* mapName)
{
  Matrix<float, Dynamic, Dynamic, ColMajor> M;
  char* fileName = (char*) malloc(4096);
  snprintf(fileName, 4095,"%s_wifi.dat", mapName);
  FILE* fid = fopen(fileName, "w");
  if(fid==NULL)
    return false;
  
  float num[] = {xMin, xMax, yMin, yMax, step};
  uint32_t numAPs = uniqueBssids.size();
  fwrite(&num[0], sizeof(float), 5, fid);
  fwrite(&numAPs, sizeof(uint32_t), 1, fid);
  fwrite(uniqueBssids.data(), sizeof(uint64_t), numAPs, fid);
  uint32_t m = apMeans[0].rows();
  uint32_t n = apMeans[0].cols();
  fwrite(&m, sizeof(uint32_t), 1, fid);
  fwrite(&n, sizeof(uint32_t), 1, fid);
  fwrite(apReadings.data(), sizeof(uint32_t), numAPs, fid);
  
  //Construct occupancy graph
  M.resize(m,n);
  for(unsigned int i=0; i<m; i++){
    for(unsigned int j=0; j<n; j++){
      float x = xMin + float(j)*step;
      float y = yMin + float(i)*step;
      Vector2f l(x,y);
      float occupancy = 0;
      for(unsigned int k=0; k<locs.size(); k++){
        if((locs[k]-l).squaredNorm()<4.0)
          occupancy ++;
      }
      M(i,j) = occupancy;
    }
  }
  fwrite(M.data(), sizeof(float), m*n, fid);
  
  //Make an empty visited occupancy graph
  //float visited[m*n];
  //fwrite(&visited[0], sizeof(float), m*n, fid);
  
  
  for(unsigned int i=0; i<numAPs; i++){
    M = apMeans[i];
    fwrite(M.data(), sizeof(float), m*n, fid);
    M = apVariances[i];
    fwrite(M.data(), sizeof(float), m*n, fid);
  }
  fclose(fid);
  return true;
}

void buildMap(const char* mapName)
{
  if(!loadData(mapName))
    std::cout << "no map" << std::endl;
    return;
  apLocs.resize(uniqueBssids.size());
  apRssiValues.resize(uniqueBssids.size());
  apKMatrices.resize(uniqueBssids.size());
  apMeans.resize(uniqueBssids.size());
  apVariances.resize(uniqueBssids.size());
  apReadings.resize(uniqueBssids.size());
  
  printf("Building K matrices...\n");
  for(unsigned int i=0; run && i<uniqueBssids.size(); i++){
    uint64_t curAP = uniqueBssids[i];
    printf("%d\n",i);
    for(unsigned int j=0; j<bssids.size(); j++){
      if(bssids[j] == curAP){
        apLocs[i].push_back(locs[j]);
        apRssiValues[i].conservativeResize(apRssiValues[i].size()+1);
        apRssiValues[i][apRssiValues[i].size()-1] = rssiValues[j];
        apReadings[i] ++;
      }
    }
    apKMatrices[i] = Kmatrix(apLocs[i]);
  }
  printf("Done.\n");
  
  printf("Generating Means and variances...\n");
  
  vector<int> numreadings(uniqueBssids.size());
  int m = ceil((yMax-yMin)/step);
  int n = ceil((xMax-xMin)/step);
  
  float mean, variance;
  for(unsigned int k=0; run && k<uniqueBssids.size(); k++){
    numreadings[k] = apLocs[k].size();
    printf("Processing AP %d, %d readings\n",k,numreadings[k]);
    apMeans[k].resize(m,n);
    apVariances[k].resize(m,n);
    for(int i=0; run && i<m; i++){
      for(int j=0; j<n; j++){
        //printf("i:%d j:%d\n",i,j);
        float x = xMin+float(j)*step;
        float y = yMin+float(i)*step;
        gp(x, y, apLocs[k], apRssiValues[k], apKMatrices[k], mean, variance);
        apMeans[k](i,j) = mean;
        apVariances[k](i,j) = variance;
      }
    }
    //fwrite(fid,mu{k},'single');
    //fwrite(fid,s{k},'single');
  }
  
  saveMap(mapName);
  printf("WiFi Map built.\n");
}

int main(int argc, char** argv) { 
  LoadParameters();
  
  char* map_name = (char*) malloc(4096);
  snprintf(map_name, 4095, "GHC7");
  
  static struct poptOption options[] = {
    { "debug",            'd', POPT_ARG_INT,     &debugLevel,         0, "Debug Level",           "NUM"},
    { "map-name",         'm', POPT_ARG_STRING , &map_name,           0, "Map name",              "STRING"},
    
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
  
  ColourTerminal(TerminalUtils::TERMINAL_COL_WHITE,TerminalUtils::TERMINAL_COL_BLACK,TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("\nWiFi Map Learning\n\n");
  ResetTerminal();
  
  InitHandleStop(&run);
  ros::init(argc, argv, "Cobot_wifi_map_learning", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  std::cout << "made it" << std::endl;
  
  buildMap(map_name);
  std::cout << "made it" << std::endl;
  
  return 0;
}
