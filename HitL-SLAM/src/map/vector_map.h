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
\file    vector_map.h
\brief   C++ Interfaces: VectorMap, LineSection
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <vector>
#include <fstream>
#include <algorithm>
#include <string>

#include "geometry.h"
#include "line.h"
#include "triangle.h"
#include "terminal_utils.h"
#include "timer.h"
#include <eigen3/Eigen/Eigen>

#ifndef VECTOR_MAP_H
#define VECTOR_MAP_H

// Vector map.
class VectorMap{

public:

  // LineSegment struct used for analytic ray casting using the map.
  typedef struct {
    double a0;
    double a1;
    int index;
    vector2f v0;
    vector2f v1;
  } LineSegment;

  // The name of the map.
  std::string mapName;

  // The directory in which the maps and the associated pre-render files are
  // stored.
  std::string mapsFolder;

  // Map Extents
  float minX, minY, maxX, maxY;

  // Number of meters per cell in visibilityList array.
  double visListResolution;

  // size of the visibilityList array
  unsigned int visListWidth, visListHeight;

  // Indicates that a pre-render file was loaded along with the map.
  bool preRenderExists;

protected:
  // The pre-rendered visibility list as a vector of the line segments
  // visible from every location.
  std::vector<std::vector<std::vector<int> > > visibilityList;

  // An empty visibility list to be returned for locations outside the map.
  std::vector<int> emptyVisibilityList;

  // The line segments that make up the map.
  std::vector<line2f> lines;

public:
  VectorMap(const std::string& _mapsFolder);
  VectorMap(const std::string& name, const std::string& _mapsFolder,
            bool usePreRender);
  ~VectorMap();

  // Get a const reference to the list of line segments in the map.
  const std::vector<line2f>& Lines() const;

  // Get a const reference to a specific line segment in the map.
  const line2f& Line(size_t i) const;

  // Replace the existing set of line segments with the vector provided.
  void updateMap(const std::vector<line2f>& new_lines);

  void sortLineSegments(
      const vector2f &loc, const std::vector<line2f> &lines,
      std::vector<LineSegment>* sorted_line_segments) const;

  // Get line which intersects first the given ray first
  int getLineCorrespondence(
      const vector2f& loc, float angle, float minRange, float maxRange,
      const std::vector< int >& visibilityList) const;

  // Get lines (for each ray) which intersect first the rays starting at
  // angles a0 to a1, at increments of da.
  std::vector<int> getRayToLineCorrespondences(
      const vector2f& loc, float angle, float a0, float a1,
      const std::vector< vector2f >& pointCloud, float minRange,
      float maxRange, bool analytical, std::vector<line2f>* lines) const;

  // Convenience function: same as previous, but specified by center angle,
  //  angle increment (da), and numRays to scan
  std::vector<int> getRayToLineCorrespondences(
      const vector2f& loc, float a0, float a1, float da,
      float minRange, float maxRange) const;

  // Return intersecting map lines for each ray generated from the provided
  // point cloud (which is assumed to be in local frame)
  std::vector<int> getRayToLineCorrespondences(
      vector2f loc, float angle, float da, int numRays, float minRange,
      float maxRange, bool analytical, std::vector<line2f>* lines) const;

  // Get ray cast from vector map at given location and angle, as specified by
  // center angle, angle increment (da), and numRays to scan
  std::vector<float> getRayCast(
      vector2f loc, float angle, float da, int numRays,
      float minRange, float maxRange);

  // Checks if any part of line2 is occluded by line1 when seen from loc, and
  // if so, line2 is trimmed accordingly, adding sub-lines to sceneLines if
  // necessary
  void trimOcclusion(
      const vector2f &loc, const line2f &line1, line2f &line2,
      std::vector<line2f> &sceneLines) const;

  // Get a set of lines which are visible from loc
  void getSceneLines(
      vector2f loc, float maxRange, std::vector<int>* sceneLinesPtr) const;

  // Saves map by name. Returns true if succesful.
  bool saveMap(const std::string& name);

  // Load map by name. Returns true if succesful.
  bool loadMap(const std::string& name, bool usePreRender);

  // Get Visibility list for specified location
  const std::vector<int>* getVisibilityList(float x, float y) const;
  const std::vector<int>* getVisibilityList(const vector2f& loc) const {
    return getVisibilityList(loc.x, loc.y);
  }

  // Perform an analytical scene render. i.e. Generate a list of lines
  // visible from loc, and the start and end angles subtended by them
  void sceneRender(
      vector2f loc,
      std::vector<line2f>* render,
      float a0 = 0.0,
      float a1 = M_2PI) const;

  // Visualize the results of an analytic ray cast from the give pose.
  void visualizeRayCast(
      const vector2f& loc,
      std::vector<float>& lines_p1x, std::vector<float>& lines_p1y,
      std::vector<float>& lines_p2x, std::vector<float>& lines_p2y,
      std::vector<uint32_t>& lines_color) const;
};

#endif //VECTOR_MAP_H
