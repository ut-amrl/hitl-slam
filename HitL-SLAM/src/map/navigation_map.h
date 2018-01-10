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
\file    navigation_map.h
\brief   C++ Interface: Vertex, Edge, Map
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#ifndef NAVIGATION_MAP_H
#define NAVIGATION_MAP_H

#include <string>
#include <vector>

#include "../shared/math/geometry.h"

class Vertex
{
  public:
    int handle;
    vector2f loc; //The co-ordinates of this vertex
    float theta;
    std::vector<int> edges;  //The indices of the edges that are adjacent to this vertex
    std::string name;
    std::string type;

  public:
    Vertex() {}
    Vertex(int _handle, float xPos, float yPos);
    Vertex(int _handle, float xPos, float yPos, float theta,
           std::string _type, std::string _name);
    void AddEdge(int edgeIndex);
    bool operator==(int _handle);
    bool operator==(std::string _name);
};

class Edge
{
  public:
    int v1; // Vertex indices, not handles
    int v2;
    float length, width;
    float orientation;
    vector2f heading;
    vector2f perp;
    float originOffset;
    float maxSpeed;
    bool doorWay;
    std::string type;

  public:
    Edge(){}
    // First two arguments are vertex indices, not handles
    Edge(int vertex1, int vertex2, Vertex &V1, Vertex &V2,
         float _width, float _maxSpeed, bool door, std::string _type);
};

class NavigationMap
{
private:
  float targetX, targetY;

  //Variables for Dijkstra's Algorithm
  //**********************************
private:
  std::vector<int> Q, subQ, QIndexes;
  std::vector<int> Qinit;
  typedef struct{
    bool operator()(int q1, int q2){ return Distance[q1]<Distance[q2]; }
  }QCompare;
  void relaxEdges(int &uIndex);
  void findSmallest(int &uIndex, int &qIndex);
public:
  std::vector<int> Policy; //Gives the next vertex to go to for each vertex, in order to reach the goal
  static std::vector<float> Distance; //From each vertex to the target
  //**********************************

public:
  std::string mapName;
  std::string mapsFolder;
  std::vector<Vertex> Vertices;
  std::vector<Edge> Edges;
  unsigned int numVertices;
  unsigned int numEdges;
  ///Number of pixels per meter
  float mapScale;
  ///Vector to add, to convert to image space
  vector2f mapOrigin;
  std::string mapImageFile;

private:
  //bool QCompare(int q1, int q2);

public:
  NavigationMap(const std::string& _mapsFolder);
  // First two arguments are vertex handles, not indices
  void AddEdge(int vertex1, int vertex2, float width, float maxSpeed,
               bool door = false, std::string type = "");
  void AddVertex(int handle, float xPos, float yPos);
  void AddVertex(int handle, float xPos, float yPos, float theta,
                 std::string type, std::string name);
  // Arguments are vertex handles, not indices
  void DeleteEdge(int vertex1, int vertex2);
  void DeleteVertex(int handle);
  // Compute Policy using Dijkstra's Algorithm
  bool ComputePolicy(float _targetX, float _targetY, bool forceReplan = false);
  void GetTarget(float &_targetX, float &_targetY);
  void GetBestProjection(float x, float y, unsigned int &edge, float &location);
  bool LoadMap(const std::string& _mapName);
  bool SaveMap(const std::string& _mapName);
  bool LoadSemanticMap(const std::string& _mapName);
  bool SaveSemanticMap(const std::string& _mapName);
  bool IsLocationAlongEdge(vector2f loc, unsigned int edge);
  float DistanceFromEdge(vector2f loc, unsigned int edge);
  float DistanceFromVertex(vector2f loc, unsigned int vertex);
  void GetProjectedLocation(vector2f loc, unsigned int &edge, float &edgeLocation, float &edgeOffset);
  vector2f GetProjectedLocation(vector2f loc);
  vector2f GetProjectedLocationOnEdge(vector2f loc, unsigned int edge);
  // Returns -1 if no vertex/edge is within thresh of loc
  int GetClosestVertex(vector2f loc, float thresh);
  int GetClosestEdge(vector2f loc, float thresh);
  int GetNextVertexIndex();
  int GetNamedVertexHandle(std::string name);
  int GetNamedVertexIndex(std::string name);
  // Returns -1 if edge is not connected to given vertex
  int otherVertexOfEdge(int edgeInd, int vertexInd);
  float angleAlongEdge(int edgeIndex, int nearVertexIndex);
};

#endif //NAVIGATION_MAP_H
