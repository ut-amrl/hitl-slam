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
\file    navigation_map.cpp
\brief   C++ Interface: Vertex, Edge, Map
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "navigation_map.h"

#include <float.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <re2/re2.h>
#include <assert.h>

#include "../shared/math/geometry.h"
#include "../shared/util/helpers.h"
#include "../shared/util/terminal_utils.h"
#include "../shared/util/timer.h"

#include <algorithm>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

using std::string;
using std::vector;
using std::ifstream;
using std::ofstream;

//========================================================================
// Vertex, Edge Classes

Vertex::Vertex(int _handle, float xPos, float yPos)
{
  loc.set(xPos,yPos);
  handle = _handle;
  theta = 0;
}
Vertex::Vertex(int _handle, float xPos, float yPos, float _theta, string _type, string _name)
{
  loc.set(xPos,yPos);
  handle = _handle;
  theta = _theta;
  type = _type;
  name = _name;
}
void Vertex::AddEdge(int edgeIndex)
{
  edges.push_back(edgeIndex);
}
bool Vertex::operator==(int _handle)
{
  return (handle == _handle);
}
bool Vertex::operator==(string _name)
{
  return (name == _name);
}


Edge::Edge(int vertex1, int vertex2, Vertex &V1, Vertex &V2,
           float _width, float _maxSpeed, bool _door=false, string _type = "")
{
  heading = (V2.loc - V1.loc).norm();
  length = (V2.loc - V1.loc).length();
  orientation = heading.angle();
  perp = heading.perp();
  originOffset = perp.dot(V1.loc);
  v1 = vertex1;
  v2 = vertex2;
  width = _width;
  maxSpeed = _maxSpeed;
  doorWay = _door;
  type = _type;
}

//========================================================================
// Navigation Map Class

vector<float> NavigationMap::Distance; //From each vertex to the target

NavigationMap::NavigationMap(const string& _mapsFolder)
{
  mapsFolder = _mapsFolder;
  mapName.clear();
  Vertices.clear();
  Edges.clear();
  numVertices = 0;
  numEdges = 0;
}

void NavigationMap::AddEdge(int vertex1, int vertex2, float width, float maxSpeed,
                            bool door, string type)
{
  static const bool debug = false;
  int v1=-1, v2=-1;
  unsigned int i = 0;
  for(i=0; i<Vertices.size(); i++){
    if(Vertices[i].handle==vertex1)
      v1=i;
    if(Vertices[i].handle==vertex2)
      v2=i;
  }
  if(v1<0||v2<0){
    TerminalWarning("Invalid vertex handles to add edge!");
    return;
  }
  int edge = -1;
  for(i=0; i<Edges.size(); i++) {
    if ((Edges[i].v1 == v1 && Edges[i].v2 == v2)
      || (Edges[i].v1 == v2 && Edges[i].v2 == v1)) {
      edge = i;
    break;
      }
  }

  if (edge != -1) {
    if (debug) TerminalWarning("Edge already exists!");
    return;
  }

  if(numEdges != Edges.size()){
    TerminalWarning("numEdges mismatch!");
    Edges.clear();
    numEdges = 0;
  }

  Edges.push_back(Edge(v1,v2,Vertices[v1],Vertices[v2],width,maxSpeed,door,type));
  Vertices[v1].AddEdge(numEdges);
  Vertices[v2].AddEdge(numEdges);
  numEdges++;
  if (debug) {
    printf("Adding edge %d-%d (handles), %fm wide %fm long\n",
           Vertices[v1].handle, Vertices[v2].handle, width,
           Edges[numEdges-1].length);
  }
}

void NavigationMap::AddVertex(int handle, float xPos, float yPos)
{
  static const bool debug = false;
  if(numVertices != Vertices.size()){
    TerminalWarning("numVertices mismatch!");
    Vertices.clear();
    numVertices = 0;
  }
  Vertices.push_back(Vertex(handle, xPos, yPos));
  if (debug) {
    printf("Adding vertex: index %i, handle %i, (%f, %f)\n",
           numVertices, handle, xPos, yPos);
  }
  numVertices++;
}

void NavigationMap::AddVertex(int handle, float xPos, float yPos, float theta, string type, string name)
{
  static const bool debug = false;
  if(numVertices != Vertices.size()){
    TerminalWarning("numVertices mismatch!");
    Vertices.clear();
    numVertices = 0;
  }
  Vertices.push_back(Vertex(handle, xPos, yPos, theta, type, name));
  if (debug) {
    printf("Adding vertex: index %i, handle %i, (%f, %f, %f)\n",
           numVertices, handle, xPos, yPos, theta);
  }
  numVertices++;
}

// Delete an edge - O(VE)
void NavigationMap::DeleteEdge(int vertex1, int vertex2)
{
  static const bool debug = false;
  unsigned int i;
  int v1 = -1, v2 = -1;
  for(i=0; i<Vertices.size(); i++) {
    if (Vertices[i].handle == vertex1) v1 = i;
    if (Vertices[i].handle == vertex2) v2 = i;
  }
  if (v1 == -1 || v2 == -1) {
    TerminalWarning("Invalid vertex handles!");
    return;
  }
  int edge = -1;
  for(i=0; i<Edges.size(); i++) {
    if ((Edges[i].v1 == v1 && Edges[i].v2 == v2)
    || (Edges[i].v1 == v2 && Edges[i].v2 == v1)) {
      edge = i;
      break;
    }
  }

  if (edge == -1) {
    TerminalWarning("Edge not found!");
    return;
  }

  if (debug) {
    printf("Deleting edge index %i, handles v1 %i, v2 %i\n",
           edge, vertex1, vertex2);
  }

  // Go through all vertices and decrement edge references
  for(i=0; i<Vertices.size(); i++) {
    for(size_t j = 0; j < Vertices[i].edges.size(); ++j) {
      // If we found the edge being deleted
      // Delete the reference
      if (Vertices[i].edges[j] == edge) {
        Vertices[i].edges.erase(Vertices[i].edges.begin() + j);
        if (debug) {
          printf("Deleting edge from vertex %i, handle %i\n", i, Vertices[i].handle);
        }
      }
      // If we found a reference to an edge after the one being delete
      // Decrement the reference
      else if (Vertices[i].edges[j] > edge) {
        Vertices[i].edges[j]--;
      }
    }
  }

  // Delete this edge
  Edges.erase(Edges.begin() + edge);
  numEdges--;
}

// Delete a vertex - O(VE)
void NavigationMap::DeleteVertex(int handle)
{
  unsigned int i;
  int v = -1;
  for(i=0; i<Vertices.size(); i++) {
    if (Vertices[i].handle == handle) {
      v = i;
      break;
    }
  }

  if (v == -1) {
    TerminalWarning(StringPrintf("No vertex with handle %i!\n", handle));
    return;
  }

  // Ensure no edge references this vertex
  for(i=0; i<Edges.size(); i++) {
    if (Edges[i].v1 == v || Edges[i].v2 == v) {
      TerminalWarning(StringPrintf("Vertex referenced by existing edge %i!", i));
      return;
    }
  }

  // Decrement vertex reference for every edge referencing vertices > this one
  for(i=0; i<Edges.size(); i++) {
    if (Edges[i].v1 > v) Edges[i].v1--;
    if (Edges[i].v2 > v) Edges[i].v2--;
  }

  // Delete this vertex
  Vertices.erase(Vertices.begin() + v);
  numVertices--;
}

void NavigationMap::GetTarget(float& _targetX, float& _targetY){
  _targetX = targetX;
  _targetY = targetY;
}

void NavigationMap::findSmallest(int &uIndex, int &qIndex)
{
  float minDist = FLT_MAX;
  for(unsigned int i=0; i<Q.size(); i++){
    int j = Q[i];
    if(minDist>Distance[j]){
      minDist = Distance[j];
      uIndex = j;
      qIndex = i;
    }
  }
}

void NavigationMap::relaxEdges(int &uIndex)
{
  Vertex &u = Vertices[uIndex];
  int v, numUEdges=u.edges.size();
  for(int i=0; i<numUEdges; i++){
    if(Edges[u.edges[i]].v1==uIndex)
      v = Edges[u.edges[i]].v2;
    else
      v = Edges[u.edges[i]].v1;

    float alt = Distance[uIndex]+Edges[u.edges[i]].length;
    if(alt<Distance[v]){
      Policy[v] = uIndex;
      Distance[v] = alt;
    }
  }
}

bool NavigationMap::ComputePolicy(float _targetX, float _targetY, bool forceReplan){
  static vector2f lastTarget(0.0,0.0);
  static const float targetChangedSqThresh = sq(0.5);
  if(false && (lastTarget-vector2f(_targetX,_targetY)).sqlength() < targetChangedSqThresh && !forceReplan){
    //Target hasn't changed, so neither should the policy
    return true;
  }
  lastTarget = vector2f(_targetX,_targetY);

  static const bool debug = false;
  targetX = _targetX;
  targetY = _targetY;

  if(Policy.size()!=numVertices)
    Policy.resize(numVertices);
  if(Distance.size()!=numVertices)
    Distance.resize(numVertices);

  for(unsigned int i=0;i<numVertices;i++){
    Distance[i] = FLT_MAX; //Some ridiculously large number
    Policy[i] = -1;
  }

  unsigned int targetEdge;
  float targetEdgeLocation;
  GetBestProjection(targetX,targetY,targetEdge,targetEdgeLocation);

  Distance[Edges[targetEdge].v1] = targetEdgeLocation;
  Distance[Edges[targetEdge].v2] = Edges[targetEdge].length-targetEdgeLocation;

  //Policy value of -2 indicates that the target location is on this edge
  Policy[Edges[targetEdge].v1] = -2;
  Policy[Edges[targetEdge].v2] = -2;

  //Dijkstra's Algorithm. For reference, See:
  // http://en.wikipedia.org/w/index.php?title=Dijkstra's_algorithm&oldid=423617835#Pseudocode
  if(Qinit.size() != numVertices){
    Qinit.clear();
    for(unsigned int i=0; i<numVertices; i++){
      Qinit.push_back(i);
    }
  }
  Q = Qinit;
  //sort(Q.begin(), Q.end(), comparer);

  while(Q.size()>0){
    int uIndex = 0, qIndex = 0;
    findSmallest(uIndex, qIndex);
    Q.erase(Q.begin()+qIndex);
    if(Distance[uIndex]==FLT_MAX){
      //Disconnected graph!
      break;
    }
    relaxEdges(uIndex);
  }

  if(debug){
    printf("\nComputed Policy:\n");
    for(unsigned int i=0;i<numVertices;i++){
      printf("%2d -> %2d %6.2f\n",i,Policy[i], Distance[i]);
    }
    printf("\n");
  }
  return true;
}

void NavigationMap::GetBestProjection(float x, float y, unsigned int &edge, float &edgeLocation)
{
  static const bool debug = false;
  edgeLocation = 0;
  edge = 0;
  int bestEdge=0;
  float bestOffset = FLT_MAX;
  float bestLocation = 0;
  vector2f loc(x,y);
  if(debug) printf("Projecting %.3f,%.3f\n",x,y);
  for(unsigned int i=0; i<Edges.size(); i++)
  {
    Edge &curEdge = Edges[i];
    Vertex &v1 = Vertices[curEdge.v1];
    Vertex &v2 = Vertices[curEdge.v2];

    float offset = distance_to_segment(v1.loc, v2.loc, loc);

    /*
    float eval1 = curEdge.heading.dot(loc-v1.loc);
    float eval2 = curEdge.heading.dot(loc-v2.loc);
    if( eval1<0 || eval2>0 )
      continue; //the point can't be projected onto this edge
      */
    edgeLocation = curEdge.heading.dot(loc-v1.loc);
    if(debug) printf("%d %.3f,%.3f:%.3f,%.3f o:%.3f d:%.3f\n",i,V2COMP(v1.loc),V2COMP(v2.loc),offset,edgeLocation);
    if(edgeLocation>curEdge.length)
      edgeLocation = curEdge.length;
    if(edgeLocation<0.0)
      edgeLocation = 0.0;

    //float offset = fabs(curEdge.perp.dot(loc-v1.loc));

    if(offset<bestOffset){
      bestOffset = offset;
      bestEdge = i;
      bestLocation = edgeLocation;
    }
  }
  edge = bestEdge;
  edgeLocation = bestLocation;
}

template <class dataType>
bool ReadRawMatrix(vector< vector<dataType> > &data, const string& fname, int numRows, int numColumns)
{
  static const bool debug = false;
  int size = numRows*numColumns*sizeof(dataType);
  int numElements = numRows*numColumns;
  dataType *buffer = (dataType *) malloc(size);
  vector <dataType> rowData;

  FILE* fp = fopen(fname.c_str(),"rb");
  if(fp==NULL){
    char buf[1025];
    if(debug){
      snprintf(buf,1024,"Unable to read from %s\n",fname.c_str());
      TerminalWarning(buf);
    }
    return false;
  }
  int sizeRead = fread(buffer, sizeof(dataType), numElements, fp);
  if(sizeRead != numElements){
    fclose(fp);
    if(debug) printf("Failed to load %s: expected:%d read:%d\n",fname.c_str(),numElements,sizeRead);
    return false;
  }
  fclose(fp);

  data.clear();
  for(int i=0;i<numRows;i++){
    rowData.clear();
    for(int j=0;j<numColumns;j++){
      rowData.push_back(buffer[i*numColumns + j]);
    }
    data.push_back(rowData);
  }
  return true;
}

template <class dataType>
bool ReadRawVector(vector<dataType> &data, char* fname, int numRows)
{
  static const bool debug = false;
  int size = numRows*sizeof(dataType);
  int numElements = numRows;
  dataType *buffer = (dataType *) malloc(size);

  FILE* fp = fopen(fname,"rb");
  if(fp==NULL){
    char buf[1025];
    if(debug) snprintf(buf,1024,"Unable to read from %s\n",fname);
    TerminalWarning(buf);
    return false;
  }
  int sizeRead = fread(buffer, sizeof(dataType), numElements, fp);
  if(sizeRead != numElements){
    fclose(fp);
    if(debug) printf("Failed to load %s: expected:%d read:%d\n",fname,numElements,sizeRead);
    return false;
  }
  fclose(fp);

  data.clear();
  for(int i=0;i<numRows;i++){
    data.push_back(buffer[i]);
  }
  return true;
}

bool NavigationMap::SaveMap(const string& _mapName)
{
  static const bool debug = false;
  static const bool debugVertices = false;
  static const bool debugEdges = false;
  const string mapDataFile = mapsFolder + "/" + _mapName + "/MapData.txt";
  const string vertexDataFile = mapsFolder + "/" + _mapName + "/MapVertices.dat";
  const string edgeDataFile = mapsFolder + "/" + _mapName + "/MapEdges.dat";

  string buf = "Saving Map " + _mapName + "\n";
  if(debug) TerminalAlert(buf);
  FILE * fp;

  fp = fopen(mapDataFile.c_str(),"w");
  if(fp==NULL){
    buf = "Unable to save to " + mapDataFile;
    TerminalWarning(buf);
    return false;
  }

  fprintf(fp,"%d\n", numVertices);
  fprintf(fp,"%d\n", numEdges);
  fprintf(fp,"%d\n", 0);
  fprintf(fp,"%f\n", -80.0);
  fprintf(fp,"%f\n", mapScale);
  fprintf(fp,"%lf\n", mapOrigin.x);
  fprintf(fp,"%lf\n", mapOrigin.y);
  fclose(fp);

  if(debug) printf(" Saving: nV:%d nE:%d nA:%d  mapScale:%f\n",numVertices,numEdges,0,mapScale);

  {
    // Save Vertices File
    int numRows = numVertices;
    int numColumns = 3;
    int numElements = numRows*numColumns;
    int size = numElements*sizeof(double);
    double *buffer = (double *) malloc(size);
    fp = fopen(vertexDataFile.c_str(),"w");
    if(fp==NULL){
      TerminalWarning("Unable to save Vertices!");
      return false;
    }
    if(debugVertices) printf("\n");
    for(int i=0; i<numRows; i++){
      buffer[i*numColumns + 0] = Vertices[i].handle;
      buffer[i*numColumns + 1] = Vertices[i].loc.x;
      buffer[i*numColumns + 2] = Vertices[i].loc.y;
      if(debugVertices) printf("Vertex %3d : %6.1f %6.1f \n",int(buffer[i*numColumns + 0]), buffer[i*numColumns + 1], buffer[i*numColumns + 2]);
    }
    fwrite(buffer, sizeof(double), numElements, fp);
    fclose(fp);
    free(buffer);
  }

  {
    // Save Edges File
    int numRows = numEdges;
    int numColumns = 5;
    int numElements = numRows*numColumns;
    int size = numElements*sizeof(double);
    double *buffer = (double *) malloc(size);
    fp = fopen(edgeDataFile.c_str(),"w");
    if(fp==NULL){
      TerminalWarning("Unable to save Vertices!");
      return false;
    }
    if(debugEdges) printf("\n");
    for(int i=0; i<numRows; i++){
      buffer[i*numColumns + 0] = Vertices[Edges[i].v1].handle;
      buffer[i*numColumns + 1] = Vertices[Edges[i].v2].handle;
      buffer[i*numColumns + 2] = Edges[i].width;
      buffer[i*numColumns + 3] = Edges[i].maxSpeed;
      buffer[i*numColumns + 4] = Edges[i].doorWay?1.0:0.0;
      if(debugEdges) printf("Edge %3d : %3d %3d \n",i, int(buffer[i*numColumns + 0]), int(buffer[i*numColumns + 1]));
    }
    fwrite(buffer, sizeof(double), numElements, fp);
    fclose(fp);
    free(buffer);
  }

  if(debug) TerminalAlert("Map Saved.");
  return true;
}

bool NavigationMap::LoadMap(const string& _mapName)
{
  static const bool debugVertices = false;
  static const bool debugEdges = false;
  static const bool debug = false;
  const string mapDataFile = mapsFolder + "/" + _mapName + "/MapData.txt";
  const string vertexDataFile = mapsFolder + "/" + _mapName + "/MapVertices.dat";
  const string edgeDataFile = mapsFolder + "/" + _mapName + "/MapEdges.dat";
  //mapImageFile = mapsFolder + "/" + string(_mapName) + "/" + string(_mapName) + ".png";

  string buf = "Loading Map " + _mapName + "\n";
  if(debug) TerminalAlert(buf);
  unsigned int numEdges, numVertices;
  FILE * fp;

  mapName = _mapName;
  fp = fopen(mapDataFile.c_str(),"r");
  if(fp==NULL){
    buf = "Unable to read from " + mapDataFile;
    TerminalWarning(buf);
    return false;
  }

  float originX, originY;
  float rfSensitivity, _mapScale;
  int numAPs;
  if( fscanf(fp,"%d", &numVertices)<1 ||
    fscanf(fp,"%d", &numEdges)<1 ||
    fscanf(fp,"%d", &numAPs)<1  ||
    fscanf(fp,"%f", &rfSensitivity)<1  ||
    fscanf(fp,"%f", &_mapScale)<1 ||
    fscanf(fp,"%f", &originX)<1  ||
    fscanf(fp,"%f", &originY)<1  ){
    //Unable to read meta data!
    fclose(fp);
    return false;
  }
  mapOrigin.set(originX,originY);
  if(debug) printf(" Loading: nV:%d nE:%d nA:%d  mapScale:%f\n",numVertices,numEdges,numAPs,_mapScale);
  fclose(fp);

  mapScale = _mapScale;

  vector< vector<double> > verticesList;
  vector< vector<double> > edgesList;

  Vertices.clear();
  Edges.clear();
  this->numEdges = this->numVertices = 0;
  if(!ReadRawMatrix(verticesList, vertexDataFile, numVertices, 3)){
    const string error =
        "ERROR:Unable to load vertices for " + _mapName;
    TerminalWarning(error.c_str());
    return false;
  }
  if(!ReadRawMatrix(edgesList, edgeDataFile, numEdges, 5)){
    const string error =
        "ERROR:Unable to load edges for " + _mapName;
    TerminalWarning(error.c_str());
    return false;
  }

  if(debug) TerminalAlert("Adding Vertices...");
  for(unsigned int i=0;i<numVertices;i++){
    if(debugVertices) printf("Vertex %d: %5.2f,%5.2f\n",int(verticesList[i][0]),verticesList[i][1],verticesList[i][2]);
    AddVertex(verticesList[i][0],verticesList[i][1],verticesList[i][2]);
  }

  if(debug) TerminalAlert("Adding Edges...");
  for(unsigned int i=0;i<numEdges;i++){
//     // Should somehow check for nonsense edges and duplicate vertices,
//     // but this doesn't properly keep the list and numEdges in sync
//     if(edgesList[i][0] == edgesList[i][1]){
//       string buf = StringPrintf("Nonsense Edge: %3d: %3d,%-3d\n",
//                                 i,int(edgesList[i][0]),int(edgesList[i][1]));
//       if(debug) TerminalWarning(buf);
//       continue;
//     }
    float maxSpeed = 0.3;
    float width = 0.6;
    if(edgesList[i].size()>2)
      width = edgesList[i][2];
    if(edgesList[i].size()>3)
      maxSpeed = edgesList[i][3];
    bool door = false;
    if(edgesList[i].size()>4)
      door = (edgesList[i][4]>FLT_MIN);
    if(debugEdges) printf("Edge %3d: %3d,%-3d width:%5.2f maxSpeed:%5.2f\n",i,int(edgesList[i][0]),int(edgesList[i][1]), width, maxSpeed);
    AddEdge(edgesList[i][0],edgesList[i][1],width, maxSpeed,door);
  }
  if(debug) TerminalAlert("Map Loaded.");
  return true;
}

bool NavigationMap::LoadSemanticMap(const string& _mapName)
{
  // TODO -- reduce amount of repeat code from LoadMap
  static const bool debugVertices = false;
  static const bool debugEdges = false;
  static const bool debug = false;
  const string mapDataFile = mapsFolder + "/" + _mapName + "/MapData.txt";
  const string vertexDataFile = mapsFolder + "/" + _mapName + "/" + _mapName + "_locations2.txt";
  const string floorEdgeDataFile = mapsFolder + "/" + _mapName + "/" + _mapName + "_edges.txt";
  const string edgeDataFile = mapsFolder + "/GHCedges.txt";

  string buf = "Loading Map " + _mapName + "\n";
  if(debug) TerminalAlert(buf);
  unsigned int numEdges, numVertices;
  FILE * fp;

  mapName = _mapName;
  fp = fopen(mapDataFile.c_str(),"r");
  if(fp==NULL){
    buf = "Unable to read from " + mapDataFile;
    TerminalWarning(buf);
    return false;
  }

  float originX, originY;
  float rfSensitivity, _mapScale;
  int numAPs;
  if( fscanf(fp,"%d", &numVertices)<1 ||
    fscanf(fp,"%d", &numEdges)<1 ||
    fscanf(fp,"%d", &numAPs)<1  ||
    fscanf(fp,"%f", &rfSensitivity)<1  ||
    fscanf(fp,"%f", &_mapScale)<1 ||
    fscanf(fp,"%f", &originX)<1  ||
    fscanf(fp,"%f", &originY)<1  ){
    //Unable to read meta data!
    fclose(fp);
    return false;
  }
  mapOrigin.set(originX,originY);
  fclose(fp);

  mapScale = _mapScale;

  Vertices.clear();
  Edges.clear();
  this->numEdges = this->numVertices = 0;

  ifstream fs;
  fs.open(vertexDataFile.c_str());
  string line, roomType, roomNumber;
  float x, y, theta;
  if (fs.is_open()) {
    RE2 re("(\\w+),\\s*(\\w+),\\s*([-]?\\d+(?:[.]\\d+)?),"
           "\\s*([-]?\\d+(?:[.]\\d+)?),\\s*([-]?\\d+(?:[.]\\d+)?)\\s*");
    while (getline(fs, line)) {
      if(RE2::FullMatch(line, re, &roomType, &roomNumber, &x, &y, &theta)) {
        AddVertex(GetNextVertexIndex(), x, y, theta, roomType, roomNumber);
        if (debugVertices) {
          printf("Vertex: %s, %s, %f, %f, %f\n", roomType.c_str(), roomNumber.c_str(), x, y, theta);
        }
      } else {
        printf("Error reading line: %s\n", line.c_str());
      }
    }
    fs.close();
  } else {
    buf = "Unable to read from " + vertexDataFile;
    TerminalWarning(buf);
    return false;
  }

  fs.open(edgeDataFile.c_str());
  string type, name1, name2;
  vector<Vertex>::iterator it;
  int v1, v2;
  if (fs.is_open()) {
    RE2 re("(\\w+)-(\\w+)-(\\w+)\\s*");
    while (getline(fs, line)) {
      if(RE2::FullMatch(line, re, &type, &name1, &name2)) {
        it = find(Vertices.begin(), Vertices.end(), name1);
        if (it != Vertices.end()) {
          v1 = (*it).handle;
        } else {
          continue;
        }
        it = find(Vertices.begin(), Vertices.end(), name2);
        if (it != Vertices.end()) {
          v2 = (*it).handle;
        } else {
          continue;
        }
        AddEdge(v1, v2, 1, 1, false, type);
        if (debugEdges) {
          printf("Edge: %s, %s (%i), %s (%i)\n",
                 type.c_str(), name1.c_str(), v1, name2.c_str(), v2);
        }
      } else {
        printf("Error reading line: %s\n", line.c_str());
      }
    }
    fs.close();
  } else {
    buf = "Unable to read from " + edgeDataFile;
    TerminalWarning(buf);
    return false;
  }
  fs.open(floorEdgeDataFile.c_str());
  if (fs.is_open()) {
    RE2 re("(\\w+)-(\\w+)-(\\w+)\\s*");
    while (getline(fs, line)) {
      if(RE2::FullMatch(line, re, &type, &name1, &name2)) {
        it = find(Vertices.begin(), Vertices.end(), name1);
        if (it != Vertices.end()) {
          v1 = (*it).handle;
        } else {
          continue;
        }
        it = find(Vertices.begin(), Vertices.end(), name2);
        if (it != Vertices.end()) {
          v2 = (*it).handle;
        } else {
          continue;
        }
        AddEdge(v1, v2, 1, 1, false, type);
        if (debugEdges) {
          printf("Edge: %s, %s (%i), %s (%i)\n",
                 type.c_str(), name1.c_str(), v1, name2.c_str(), v2);
        }
      } else {
        printf("Error reading line: %s\n", line.c_str());
      }
    }
    fs.close();
  } else {
    buf = "Unable to read from " + floorEdgeDataFile;
    TerminalWarning(buf);
    return false;
  }
  return true;
}

bool NavigationMap::SaveSemanticMap(const string& _mapName)
{
  static const bool debug = false;
  static const bool debugVertices = false;
  static const bool debugEdges = false;
  const string vertexDataFile = mapsFolder + "/" + _mapName + "/" + _mapName + "_locations2.txt";
  const string edgeDataFile = mapsFolder + "/" + _mapName + "/" + _mapName + "_edges.txt";

  string buf = "Saving Map " + _mapName + "\n";
  if(debug) TerminalAlert(buf);
  ofstream fs;

  // Save Vertices File
  fs.open(vertexDataFile.c_str());
  if(!fs.is_open()){
    TerminalWarning("Unable to save Vertices!");
    return false;
  }
  if(debugVertices) printf("\n");
  for(size_t i=0; i<numVertices; i++){
    const Vertex& v = Vertices[i];
    fs << v.type << "," << v.name << "," << v.loc.x << "," << v.loc.y << "," << v.theta << "\n";
    if(debugVertices) {
      printf("Vertex %s (%s): (%f, %f, %f)\n",
             v.name.c_str(), v.type.c_str(), v.loc.x, v.loc.y, v.theta);
    }
  }
  fs.close();

  // Save Edges File
  fs.open(edgeDataFile.c_str());
  if(!fs.is_open()){
    TerminalWarning("Unable to save Edges!");
    return false;
  }
  if(debugEdges) printf("\n");
  for(size_t i=0; i<numEdges; i++){
    string v1name = Vertices[Edges[i].v1].name;
    string v2name = Vertices[Edges[i].v2].name;
    fs << Edges[i].type << "-" << v1name << "-" << v2name << "\n";
    if(debugEdges) printf("Edge %s: %s-%s\n", Edges[i].type.c_str(), v1name.c_str(), v2name.c_str());
  }
  fs.close();

  if(debug) TerminalAlert("Map Saved.");
  return true;
}

float NavigationMap::DistanceFromEdge(vector2f loc, unsigned int edge)
{
  const line2f line(Vertices[Edges[edge].v1].loc, Vertices[Edges[edge].v2].loc);
  return (line.closestDistFromLine(loc, true));
  // return fabs(Edges[edge].perp.dot(loc-Vertices[Edges[edge].v1].loc));
}

float NavigationMap::DistanceFromVertex(vector2f loc, unsigned int vertex)
{
  return (Vertices[vertex].loc - loc).length();
}


bool NavigationMap::IsLocationAlongEdge(vector2f loc, unsigned int edge)
{
  const vector2f &v1 = Vertices[Edges[edge].v1].loc;
  const vector2f &v2 = Vertices[Edges[edge].v2].loc;
  return (loc-v1).dot(loc-v2) < 0.0;
}

vector2f NavigationMap::GetProjectedLocation(vector2f loc)
{
  unsigned int edge;
  float edgeLocation;
  GetBestProjection(loc.x,loc.y,edge,edgeLocation);
  //printf("e:%d v1:%d loc:%f\n",edge,(int)Edges.size(),edgeLocation);
  return Vertices[Edges[edge].v1].loc + Edges[edge].heading*edgeLocation;
}

void NavigationMap::GetProjectedLocation(vector2f loc, unsigned int& edge, float& edgeLocation, float& edgeOffset)
{
  GetBestProjection(loc.x,loc.y,edge,edgeLocation);
  edgeOffset = Edges[edge].perp.dot(loc-Vertices[Edges[edge].v1].loc);
}

vector2f NavigationMap::GetProjectedLocationOnEdge(vector2f loc, unsigned int edge)
{
  Edge curEdge = Edges[edge];
  Vertex v1 = Vertices[curEdge.v1];
  Vertex v2 = Vertices[curEdge.v2];

//   float offset = distance_to_segment(v1.loc, v2.loc, loc);

  float edgeLocation = curEdge.heading.dot(loc-v1.loc);
  //if(debug) printf("%d %.3f,%.3f:%.3f,%.3f o:%.3f d:%.3f\n",i,V2COMP(v1.loc),V2COMP(v2.loc),offset,edgeLocation);
  if(edgeLocation>curEdge.length)
    edgeLocation = curEdge.length;
  if(edgeLocation<0.0)
    edgeLocation = 0.0;

  return Vertices[Edges[edge].v1].loc + Edges[edge].heading*edgeLocation;
}

int NavigationMap::GetClosestVertex(vector2f loc, float thresh)
{
  // Returns -1 if no vertex is within thresh of loc
  float min_distance = FLT_MAX;
  size_t best_vertex = 0;
  for (size_t i = 0; i < numVertices; ++i) {
    float distance = DistanceFromVertex(loc, i);
    if (distance < min_distance) {
      min_distance = distance;
      best_vertex = i;
    }
  }
  if (min_distance < thresh) {
    return static_cast<int>(best_vertex);
  } else {
    return -1;
  }
}

int NavigationMap::GetClosestEdge(vector2f loc, float thresh)
{
  // Returns -1 if no vertex is within thresh of loc
  float min_distance = FLT_MAX;
  size_t best_edge = 0;
  for (size_t i = 0; i < numEdges; ++i) {
    float distance = DistanceFromEdge(loc, i);
    if (distance < min_distance) {
      min_distance = distance;
      best_edge = i;
    }
  }
  if (min_distance < thresh) {
    return static_cast<int>(best_edge);
  } else {
    return -1;
  }
}

int NavigationMap::GetNextVertexIndex()
{
  bool vertex_exists = false;
  int next_vertex_id = 0;
  do {
    vertex_exists = false;
    for (size_t i = 0; i < numVertices; ++i) {
      if (next_vertex_id == Vertices[i].handle) {
        vertex_exists = true;
        break;
      }
    }
    if (vertex_exists) ++next_vertex_id;
  } while(vertex_exists);
  return next_vertex_id;
}

int NavigationMap::GetNamedVertexHandle(string name)
{
  int vertex_handle = -1;
  for (size_t i = 0; i < numVertices; ++i) {
    if (Vertices[i].name.length() > 0 && Vertices[i].name == name) {
      vertex_handle = Vertices[i].handle;
      break;
    }
  }
  return vertex_handle;
}

int NavigationMap::GetNamedVertexIndex(string name)
{
  int vertex_index = -1;
  for (size_t i = 0; i < numVertices; ++i) {
    if (Vertices[i].name.length() > 0 && Vertices[i].name == name) {
      vertex_index = i;
      break;
    }
  }
  return vertex_index;
}

// Given the vertex at one end of an edge, get index of the one at the other end
int NavigationMap::otherVertexOfEdge(int edgeInd, int vertexInd) {
  int otherVertexInd = -1;
  Edge e = Edges[edgeInd];
  if (e.v1 == vertexInd) {
    otherVertexInd = e.v2;
  } else if (e.v2 == vertexInd) {
    otherVertexInd = e.v1;
  } else {
    printf("Edge %i is not attached to vertex %i!\n", edgeInd, vertexInd);
  }
  return otherVertexInd;
}

// Compute the angle (in global frame) of the path along an edge of the map
// in a particular direction, given by the starting vertex.  Assumes edge is
// connected to vertex.
float NavigationMap::angleAlongEdge(int edgeIndex, int nearVertexIndex) {
  Edge e = Edges[edgeIndex];
  Vertex nearV = Vertices[nearVertexIndex];
  Vertex farV = Vertices[otherVertexOfEdge(edgeIndex, nearVertexIndex)];

  return (farV.loc - nearV.loc).angle();
}

