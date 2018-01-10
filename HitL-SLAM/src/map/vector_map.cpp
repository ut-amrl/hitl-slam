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
\file    vector_map.cpp
\brief   C++ Implementation: VectorMap, LineSection
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "vector_map.h"

#include <vector>
#include <string>
#include <locale.h>
#include "helpers.h"

using std::size_t;
using std::string;
using std::vector;

const vector<line2f>& VectorMap::Lines() const {
  return lines;
}

const line2f& VectorMap::Line(size_t i) const {
  return lines[i];
}

bool VectorMap::saveMap(const string& name) {
  static const float sqeps = sq(0.001);
  static const float minlength = 0.1;
  for (size_t i = 0; i < lines.size(); ++i) {
    if (lines[i].Length() < minlength) {
      lines.erase(lines.begin() + i);
      --i;
      continue;
    }
    const vector2f& a0 = lines[i].P0();
    const vector2f& a1 = lines[i].P1();
    for (size_t j = i + 1; j < lines.size(); ++j) {
      const vector2f& b0 = lines[j].P0();
      const vector2f& b1 = lines[j].P0();
      if (((a0 - b0).sqlength() <= sqeps && (a1 - b1).sqlength() <= sqeps) ||
        ((a0 - b1).sqlength() <= sqeps && (a1 - b0).sqlength() <= sqeps)) {
        lines.erase(lines.begin() + j);
        --j;
      }
    }
  }

  mapName = name;
  char vectorFile[4096];
  snprintf(vectorFile, 4095, "%s/%s/%s_vector.txt",
           mapsFolder.c_str(), name.c_str(), name.c_str());
  printf("Saving map: %s\n", name.c_str());
  ScopedFile file(vectorFile, "w");
  for (size_t i = 0; i < lines.size(); ++i) {
    fprintf(file(), "%.4f,%.4f,%.4f,%.4f\n",
            V2COMP(lines[i].P0()), V2COMP(lines[i].P1()));
  }
  return true;
}

bool VectorMap::loadMap(const string& name, bool usePreRender) {
  static const bool debug = false;
  static const bool debugVector = false;

  char vectorFile[4096];
  char renderFile[4096];

  mapName = name;
  snprintf(vectorFile, 4095, "%s/%s/%s_vector.txt",
           mapsFolder.c_str(), name.c_str(), name.c_str());
  snprintf(renderFile, 4095, "%s/%s/%s_render.dat",
           mapsFolder.c_str(), name.c_str(), name.c_str());
  if (debug) printf("Loading map: %s\n", name.c_str());

  //Read Vector data
  if (debug) printf("Loading vector map from %s\n",vectorFile);
  ScopedFile fid_vector(vectorFile,"r");
  if (fid_vector() == NULL) {
    char buf[1024];
    snprintf(buf, 1023, "Unable to load vector file %s!",vectorFile);
    TerminalWarning(buf);
    return false;
  }
  float x1,y1,x2,y2;
  minX = minY = FLT_MAX;
  maxX = maxY = -FLT_MAX;
  lines.clear();
  while(fscanf(fid_vector(),"%f,%f,%f,%f",&x1,&y1,&x2,&y2)==4) {
    if (debugVector) {
      printf("Line%d: <%f %f> <%f %f>\n",(int)lines.size(),x1,y1,x2,y2);
    }
    minX = min(minX,x1);
    minX = min(minX,x2);
    minY = min(minY,y1);
    minY = min(minY,y2);
    maxX = max(maxX,x1);
    maxX = max(maxX,x2);
    maxY = max(maxY,y1);
    maxY = max(maxY,y2);
    vector2f p0(x1,y1);
    vector2f p1(x2,y2);

    lines.push_back(line2f(p0,p1));
    lines.at(lines.size()-1).calcValues();
  }
  if (debug) printf("%d lines loaded into vector map\n",(int)lines.size());
  if (debug) printf("Extents: %.3f,%.3f : %.3f,%.3f\n",minX, minY, maxX, maxY);

  //Read Pre-Render data
  preRenderExists = false;
  visibilityList.clear();
  if (usePreRender) {
    ScopedFile fid_render(renderFile, "r");
    if (fid_render() == NULL) {
      char buf[1024];
      snprintf(buf, 1023, "Unable to load pre-render file %s!",renderFile);
      TerminalWarning(buf);
    } else {
      bool error = false;
      int x=0, y=0;
      unsigned int cnt = 0;
      error = fread(&visListWidth,sizeof(unsigned int),1,fid_render())!=1;
      error = error ||
          (fread(&visListHeight,sizeof(unsigned int),1,fid_render())!=1);
      error = error ||
          (fread(&visListResolution,sizeof(double),1,fid_render())!=1);
      if (!error) {
        if (debug) {
          printf("Pre-render size: %d x %d, resolution: %.3f\n",
                 visListWidth, visListHeight, visListResolution);
        }
        visibilityList.resize(visListWidth);
        for(unsigned int i=0; i<visListWidth; i++)
          visibilityList[i].resize(visListHeight);
      }
      while(cnt<visListHeight*visListWidth && !error) {
        size_t size = 0;
        error = fread(&x,sizeof(int),1,fid_render())!=1;
        error = error || (fread(&y,sizeof(int),1,fid_render())!=1);
        error = error || (fread(&size,sizeof(int),1,fid_render())!=1);

        if (x<0 || y<0 ||
            x > static_cast<int>(visListWidth) - 1 ||
            y > static_cast<int>(visListHeight) - 1) {
          char msg[4096];
          snprintf(msg, 4095, "Invalid loc (%d,%d) in pre-render file",x,y);
          TerminalWarning(msg);
          error = true;
        }
        visibilityList[x][y].resize(size);

        error = error ||
            (fread(visibilityList[x][y].data(),sizeof(int),size,fid_render()) !=
            size);

        if (!error && debug && ((y+1)*100)%visListHeight==0) {
          int progress = (y+1)*100/visListHeight;
          printf("\rReading pre-render file... %3d%% ",progress);
          fflush(stdout);
        }
        if (!error)
          cnt++;
      }
      if (error) {
        char buf[1024];
        snprintf(buf, 1023, "\nUnable to parse pre-render file %s",renderFile);
        TerminalWarning(buf);
        preRenderExists = false;
      } else {
        if (debug) printf("\nRead %d locations into visibility list\n",cnt);
        preRenderExists = true;
      }
    }
  }

  if (debug) printf("Done loading map\n\n");
  return true;
}

VectorMap::VectorMap(const string& _mapsFolder) :
    mapsFolder(_mapsFolder), minX(0.0), minY(0.0), maxX(0.0), maxY(0.0),
    visListResolution(0.0), visListWidth(0), visListHeight(0),
    preRenderExists(false) {
  if (setlocale(LC_ALL, "en_US.UTF-8") == NULL) {
    fprintf(stderr,
            "WARNING: Unable to set locale. VectorMap might have trouble"
            "parsing map files.\n");
  }
}

VectorMap::VectorMap(
    const string& name, const string& _mapsFolder, bool usePreRender) :
    mapsFolder(_mapsFolder), minX(0.0), minY(0.0), maxX(0.0), maxY(0.0),
    visListResolution(0.0), visListWidth(0), visListHeight(0),
    preRenderExists(false) {
  loadMap(name, usePreRender);
  //initOpenGL();
}

VectorMap::~VectorMap() {
}

const std::vector<int>* VectorMap::getVisibilityList(float x, float y) const {
  const int xInd = static_cast<int>(
      floor((x - minX) / visListResolution + 0.5));
  const int yInd = static_cast<int>(
      floor((y - minY) / visListResolution + 0.5));
  if (xInd < 0 || xInd > static_cast<int>(visListWidth) - 1 ||
      yInd < 0 || yInd > static_cast<int>(visListHeight) - 1) {
    return (&emptyVisibilityList);
  }
  DCHECK_GE(xInd, 0);
  DCHECK_GE(yInd, 0);
  DCHECK_LT(xInd, visListWidth);
  DCHECK_LT(yInd, visListHeight);
  return &visibilityList[xInd][yInd];
}

vector<float> VectorMap::getRayCast(vector2f loc, float angle,
                                    float da, int numRays,
                                    float minRange, float maxRange) {
  static const bool UsePreRender = true;
  float intervals = numRays - 1.0;
  float a0 = angle - 0.5*intervals*da;
  float a1 = angle + 0.5*intervals*da;
  vector<float> rayCast;
  rayCast.clear();

  for(float a=a0; a<a1; a += da) {
    float ray = maxRange;
    if (preRenderExists && UsePreRender) {
      const vector<int>* visibilityList = getVisibilityList(loc);
      for(unsigned int i=0; i<visibilityList->size(); i++) {
        int lineIndex = visibilityList->at(i);
        float curRay = maxRange;
        if (lines[lineIndex].intersects((vector2f)loc,(float)a)) {
          line2f &l = lines[lineIndex];
          //curRay = l.distFromLine1((vector2f)loc,(double)a, false);
          {
            vector2f heading;
            heading.heading(a);
            if (l.intersects(loc,heading,true)) {
              float sinTheta = l.Dir().cross(heading.norm());
              curRay = fabs(l.Perp().dot(loc-l.P0())/sinTheta);
            } else {
              curRay = NAN;
            }
          }
          if (curRay<0.0) {
            TerminalWarning("Ray Cast Fail!");
            printf("line: %.3f,%.3f : %.3f,%.3f loc:%.3f,"
                   "%.3f a:%.1f\u00b0 curRay:%f\n",
                   V2COMP(lines[lineIndex].P0()),
                   V2COMP(lines[lineIndex].P1()),
                   V2COMP(loc),DEG(a),curRay);
            fflush(stdout);
            while(1) {
              Sleep(0.05);
            }
            //exit(0);
            //alarm(1);
          }
          ray = min(ray, curRay);
        }
      }
    } else {
      vector<int> sceneLines;
      getSceneLines(loc, maxRange, &sceneLines);
      for(unsigned int i=0; i<sceneLines.size(); i++) {
        if (lines[sceneLines[i]].intersects(loc,a)) {
          ray = min(ray, lines[sceneLines[i]].distFromLine1(
              (vector2f)loc,(float)a, false));
        }
      }
    }
    rayCast.push_back(ray);
  }
  return rayCast;
}

int VectorMap::getLineCorrespondence(
    const vector2f& loc, float angle, float minRange, float maxRange,
    const vector<int>& visibilityList) const {
  vector2f loc1, loc2, dir;
  dir.heading(angle);
  loc1 = loc + minRange*dir;
  loc2 = loc + maxRange*dir;

  vector2f p = loc2;
  int bestLine = -1;
  for(unsigned int i=0; i<visibilityList.size(); i++) {
    int lineIndex = visibilityList[i];
    if (!lines[lineIndex].intersects(loc1,loc2,false,false,true))
      continue;

    vector2f p2 = lines[lineIndex].intersection(loc1,loc2,false,false);
    if ((p2-loc).sqlength()<(p-loc).sqlength()) {
      p = p2;
      bestLine = lineIndex;
    }
  }
  return bestLine;
}

vector<int> VectorMap::getRayToLineCorrespondences(
    const vector2f& loc, float angle, float a0, float a1,
    const vector<vector2f>& pointCloud, float minRange,
    float maxRange, bool analytical, vector<line2f> *lines ) const {
  //FunctionTimer ft(__FUNCTION__);
  static const bool UsePreRender = true;

  vector<int> correspondences;
  correspondences.clear();
  const vector<int>* preRenderVisibilityList = NULL;
  vector<int> visibilityList;
  if (UsePreRender && preRenderExists) {
    preRenderVisibilityList = getVisibilityList(loc);
  } else {
    getSceneLines(loc,maxRange, &visibilityList);
  }
  const vector<int>& locVisibilityList =
      (UsePreRender && preRenderExists) ?
      (*preRenderVisibilityList) : visibilityList;

  if (analytical && lines!=NULL) {
    sceneRender(loc, lines, a0, a1);
    vector<LineSegment> segments;
    sortLineSegments(loc,*lines, &segments);
    float rotMat1[4] = {
        static_cast<float>(cos(angle)),
        static_cast<float>(-sin(angle)),
        static_cast<float>(sin(angle)),
        static_cast<float>(cos(angle))
    };
    vector2f p(0.0,0.0);
    correspondences.resize(pointCloud.size());
    for(unsigned int i=0; i<pointCloud.size(); i++) {
      p = pointCloud[i];
      p.set(p.x*rotMat1[0] + p.y*rotMat1[1], p.x*rotMat1[2] + p.y*rotMat1[3]);
      int correspondence = -1;

      for(unsigned int j=0; j<segments.size() && correspondence<0; j++) {
        if (segments[j].v0.cross(p)>=0.0f && segments[j].v1.cross(p)<=0.0f)
          correspondence = segments[j].index;
      }
      correspondences[i] = correspondence;
    }
  } else {
    for(unsigned int i=0; i<pointCloud.size(); i++) {
      float curAngle = angle_mod(pointCloud[i].angle() + angle);
      correspondences.push_back(
          getLineCorrespondence(loc,curAngle,minRange,
                                maxRange, locVisibilityList));
    }
  }
  return correspondences;
}

vector<int> VectorMap::getRayToLineCorrespondences(
    const vector2f& loc, float a0, float a1, float da,
    float minRange, float maxRange) const {
  //FunctionTimer ft(__PRETTY_FUNCTION__);
  vector<int> correspondences;
  correspondences.clear();

  const vector<int>* preRenderVisibilityList = NULL;
  vector<int> visibilityList;
  if (preRenderExists) {
    preRenderVisibilityList = getVisibilityList(loc);
  } else {
    getSceneLines(loc,maxRange, &visibilityList);
  }
  const vector<int>& locVisibilityList =
      (preRenderExists) ? (*preRenderVisibilityList) : visibilityList;

  for(float a=a0; a<a1; a += da) {
    correspondences.push_back(
        getLineCorrespondence(loc,a,minRange, maxRange, locVisibilityList));
  }
  return correspondences;
}

void VectorMap::sortLineSegments(
    const vector2f& loc, const vector<line2f>& lines,
    vector<LineSegment>* sorted_line_segments_ptr) const {
  static const float eps = RAD(0.001);
  vector<VectorMap::LineSegment>& segments = *sorted_line_segments_ptr;
  segments.clear();
  for(unsigned int i=0; i<lines.size(); i++) {
    float a0 = angle_pos((lines[i].P0()-loc).angle());
    float a1 = angle_pos((lines[i].P1()-loc).angle());
    if ((lines[i].P0()-loc).cross(lines[i].P1()-loc)<0.0)
      swap(a0,a1);
    LineSegment curSegment;
    curSegment.a0 = a0;
    curSegment.a1 = a1;
    curSegment.index = i;
    curSegment.v0.heading(a0);
    curSegment.v1.heading(a1);

    if (segments.size()<1) {
      segments.push_back(curSegment);
      continue;
    }

    // Special case: line intersects the x axis, so curSegment MUST be the
    // first segment.
    if (a0>a1) {
      segments.insert(segments.begin(), curSegment);
      continue;
    }
    unsigned int j=0;
    bool found = false;
    for(; j<segments.size()-1 && !found; j++) {
      if (segments[j].a1<=a0+eps && segments[j+1].a0>=a1-eps )
        found = true;
    }

    if (found && j<segments.size())
      segments.insert(segments.begin()+j,curSegment);
    else
      segments.push_back(curSegment);
  }

  if (segments.size()>0) {
    if (segments[0].a0>segments[0].a1) {
      LineSegment curSegment;
      curSegment.a0 = segments[0].a0;
      curSegment.a1 = M_2PI;
      curSegment.index = segments[0].index;
      segments[0].a0 = 0.0;
      segments.push_back(curSegment);
    }
  }
}

vector<int> VectorMap::getRayToLineCorrespondences(
    vector2f loc, float angle, float da, int numRays,
    float minRange, float maxRange, bool analytical,
    vector<line2f> *lines) const {
  //FunctionTimer ft("getRayToLineCorrespondences");
  float intervals = numRays - 1.0;
  float a0 = angle - 0.5*intervals*da;
  float a1 = angle + 0.5*intervals*da;
  static const float kAngularMargin = RAD(1.0);
  if (analytical && (lines!=NULL)) {
    //FunctionTimer ft("Analytic Render");
    a0 = angle_pos(a0);
    sceneRender(loc, lines, a0, a1);
    vector<VectorMap::LineSegment> segments;
    sortLineSegments(loc,*lines, &segments);
    vector<int> correspondences;
    correspondences.clear();
    for(int i=0; i<numRays; i++) {
      correspondences.push_back(-1);
    }
    int maxScanRays = floor((float)M_2PI/da);

    for(unsigned int i=0; i<segments.size(); i++) {
      float aStart = segments[i].a0;
      float aEnd = segments[i].a1;
      if (angle_dist(aStart, aEnd) < 2.0 * kAngularMargin) {
        continue;
      }
      aStart += kAngularMargin;
      aEnd -= kAngularMargin;
      if (aEnd<aStart)
        aEnd += M_2PI;
      int index = ceil((float) (aStart - a0)/da );
      if (index<0)
        index+=maxScanRays;
      for(float a = aStart; a<aEnd; a+=da, index++) {
        int j = index%maxScanRays;
        if (j<numRays) {
          correspondences[j] = segments[i].index;
        }
      }
    }

    return correspondences;
  } else
    return getRayToLineCorrespondences(loc,a0,a1,da,minRange,maxRange);
}

void VectorMap::getSceneLines(
    vector2f loc, float maxRange, vector<int>* sceneLinesPtr) const {
  static const float eps = 1e-4;
  vector<int> linesList;
  vector<int>& sceneLines = *sceneLinesPtr;
  sceneLines.clear();
  for(unsigned int i=0; i<lines.size(); i++) {
    if (lines[i].closestDistFromLine(loc, true)<maxRange)
      linesList.push_back(i);
  }

  vector<line2f> tmpList;
  for(unsigned int i=0; i<linesList.size(); i++) {
    tmpList.clear();
    tmpList.push_back(lines[linesList[i]]);
    // Check if any part of curLine is unoccluded by present list of lines,
    // as seen from loc.
    bool visible = false;
    for (size_t j = 0; j < tmpList.size(); ++j) {
      line2f curLine = tmpList[j];
      for (size_t k = 0; k < linesList.size() && curLine.Length() > eps; ++k) {
        if (i==k) {
          // Don't check for self-occlusions!
          continue;
        }
        if (lines[linesList[k]].Length() <= eps)
          continue;
        trimOcclusion(loc, lines[linesList[k]], curLine, tmpList);
      }
      if (curLine.Length() > eps) {
        // At least part of curLine is unoccluded.
        visible = true;
      }
    }
    if (visible) {
      sceneLines.push_back(linesList[i]);
    }
  }
}

void VectorMap::trimOcclusion(
    const vector2f& loc, const line2f& line1,
    line2f& line2, vector<line2f>& sceneLines) const {
  // Checks if any part of line2 is occluded by line1 when seen from loc, and
  // if so, line2 is trimmed accordingly, adding sub-lines to sceneLines if
  // necessary
  static const float eps = 1e-4;
  static const float sqeps = 1e-8;
  if (line1.Length()<eps || line2.Length()<eps)
    return;

  vector2f l1_p0 = line1.P0();
  vector2f l1_p1 = line1.P1();
  vector2f l1_r0 = l1_p0-loc;
  vector2f l1_r1 = l1_p1-loc;
  vector2f l2_p0 = line2.P0();
  vector2f l2_p1 = line2.P1();
  vector2f l2_r0 = l2_p0-loc;
  vector2f l2_r1 = l2_p1-loc;

  //Ensure that r0 vector to r1 vector is in the positive right-handed order
  if (l1_r0.cross(l1_r1)<0.0 ) {
    swap(l1_r0,l1_r1);
    swap(l1_p0,l1_p1);
  }
  if (l2_r0.cross(l2_r1)<0.0 ) {
    swap(l2_r0,l2_r1);
    swap(l2_p0,l2_p1);
  }

  if ((l1_r0.cross(l2_r0) >= 0.0 && l1_r1.cross(l2_r0) >= 0.0) ||
      (l2_r1.cross(l1_r0) >= 0.0 && l2_r1.cross(l2_r1) >= 0.0)) {
    // No Line interaction.
    return;
  }

  // The lines line1 and line2 intersect each other.
  const bool intersects = line2.intersects(line1,false,false,false);

  // The semi-infinite ray from loc and passing through line1.p0 intersects
  // line2.
  const bool rayOcclusion1 = line2.intersects(loc,l1_r0, false);
  // The semi-infinite ray from loc and passing through line1.p1 intersects
  // line2.
  const bool rayOcclusion2 = line2.intersects(loc,l1_r1, false);

  vector2f p;
  if (intersects) {
    vector2f mid = line2.intersection(line1,false,false);
    if ((l1_p0-mid).cross(l2_p0-mid) > 0.0) {
      // Delete the right hand part of line2.
      line2 = line2f(mid,l2_p1);
      line2f l(mid, l2_p0);
      if (l.intersects(loc,l1_r0,false)) {
        p = l.intersection(loc,l1_p0,false, true);
        if ((l2_p0-p).sqlength() > sqeps) {
          // Part of the right hand part of line2 extends beyond line1, it may
          // still be visible.
          sceneLines.push_back(line2f(l2_p0, p));
        }
      }
    } else {
      // Delete the left hand part of line2.
      line2 = line2f(l2_p0,mid);
      line2f l(mid, l2_p1);
      if (l.intersects(loc,l1_r1,false)) {
        p = l.intersection(loc,l1_p1,false, true);
        if ((l2_p1-p).sqlength()>sqeps) {
          // Part of the left hand part of line2 extends beyond line1, it may
          // still be visible.
          sceneLines.push_back(line2f(l2_p1, p));
        }
      }
    }
  } else {
    const bool completeOcclusion =
        line1.intersects(line2f(loc,l2_p0),false, false, true) &&
        line1.intersects(line2f(loc,l2_p1),false, false, true);

    // line1.p0 is in front of, and occludes line2.
    const bool occlusion0 = rayOcclusion1 &&
        !line2.intersects(line2f(loc,l1_p0), false, false, false);

    // line1.p1 is in front of, and occludes line2.
    const bool occlusion1 = rayOcclusion2 &&
        !line2.intersects(line2f(loc,l1_p1), false, false, false);

    if (completeOcclusion) {
      // Trim the line to zero length.
      line2 = line2f(0.1,0.1,0.1,0.1);
    } else if (occlusion0 && occlusion1) {
      // line2 is partially occluded in the middle by line1. Break up into 2
      // segments, make line2 one segment, push back the other segment to the
      // sceneLines list.
      const vector2f right_section_end =
          line2.intersection(line2f(loc,l1_p0),false, true);
      const vector2f left_section_end =
          line2.intersection(line2f(loc,l1_p1),false, true);
      line2.set(l2_p0, right_section_end);
      // save the unoccluded part of line2 at its left hand end, if any
      if ((left_section_end - l2_p1).sqlength()>sqeps)
        sceneLines.push_back(line2f(left_section_end, l2_p1));
    } else if (occlusion0) {
      //The left hand end of line2 is occluded, trim it
      vector2f right_section_end =
          line2.intersection(line2f(loc,l1_p0),false, true);
      line2.set(l2_p0, right_section_end);
    } else if (occlusion1) {
      //The right hand end of line2 is occluded, trim it
      vector2f left_section_end =
          line2.intersection(line2f(loc,l1_p1),false, true);
      line2.set(left_section_end, l2_p1);
    }
  }
}

void VectorMap::sceneRender(
    vector2f loc, std::vector<line2f>* render, float a0, float a1) const {
  //FunctionTimer ft("Scene Render");
  static const float eps = 0.001;
  static const float MaxRange = 5.0;
  static const unsigned int MaxLines = 200;

  vector2f leftMargin, rightMargin;
  rightMargin.heading(a0);
  leftMargin.heading(a1);

  vector<line2f> scene;
  vector<line2f> linesList;

  scene.clear();
  render->clear();
  linesList.clear();

  const vector<int>* preRenderVisibilityList = NULL;
  vector<int> visibilityList;
  if (preRenderExists) {
    preRenderVisibilityList = getVisibilityList(loc);
  } else {
    getSceneLines(loc, MaxRange, &visibilityList);
  }
  const vector<int>& locVisibilityList =
      (preRenderExists) ? (*preRenderVisibilityList) : visibilityList;

  linesList.resize(locVisibilityList.size());
  for(unsigned int i=0; i<locVisibilityList.size(); i++) {
    linesList[i] = lines[locVisibilityList[i]];
  }
  unsigned int i, j;
  for(i=0; i<linesList.size() && i<MaxLines; i++) {
    line2f curLine = linesList[i];
    // Check if any part of curLine is unoccluded by present list of lines,
    // as seen from loc
    for(j=0;j<scene.size() && curLine.Length()>=eps; j++) {
      if (scene[j].Length()<eps)
        continue;
      trimOcclusion(loc, scene[j], curLine,linesList);
    }

    if (curLine.Length()>eps ) { //At least part of curLine is unoccluded
      for(j=0; j<scene.size(); j++) {
        if (scene[j].Length()<eps)
          continue;
        trimOcclusion(loc, curLine, scene[j],linesList);
      }
      //add the visible part of curLine
      scene.push_back(curLine);
    }
  }

  if (linesList.size()>=MaxLines) {
    char buf[2048];
    sprintf(buf,
            "Runaway Analytic Scene Render at %.30f,%.30f, %.3f : %.3f\u00b0",
            V2COMP(loc),DEG(a0),DEG(a1));
    TerminalWarning(buf);
  }
  for(i=0; i<scene.size(); i++) {
    if (scene[i].Length() > eps)
      render->push_back(scene[i]);
  }
}

void VectorMap::updateMap(const vector<line2f>& new_lines) {
  lines = new_lines;
  CHECK_GT(lines.size(), 0);
  // Since the map was just updated, the pre-render list is now outdated.
  preRenderExists = false;
  // Recompute map extents.
  minX = FLT_MAX;
  maxX = -FLT_MAX;
  minY = FLT_MAX;
  maxY = -FLT_MAX;
  for (size_t i = 0; i < lines.size(); ++i) {
    minX = min(minX, lines[i].P0().x);
    minX = min(minX, lines[i].P1().x);
    maxX = max(maxX, lines[i].P0().x);
    maxX = max(maxX, lines[i].P1().x);

    minY = min(minY, lines[i].P0().y);
    minY = min(minY, lines[i].P1().y);
    maxY = max(maxY, lines[i].P0().y);
    maxY = max(maxY, lines[i].P1().y);
  }
}

void VectorMap::visualizeRayCast(
    const vector2f& loc,
    vector<float>& lines_p1x, vector<float>& lines_p1y,
    vector<float>& lines_p2x, vector<float>& lines_p2y,
    vector<uint32_t>& lines_color) const {
  const float MaxRange = 6.5;
  static const int LineExtentColor = 0xFFFFD659;
  static const int DebugColour = 0xFFFF0000;

  vector<line2f> scene_lines;
  sceneRender(loc, &scene_lines, 0.0, RAD(359.9));
  const vector<int>* preRenderVisibilityList = NULL;
  vector<int> visibilityList;
  if (preRenderExists) {
    preRenderVisibilityList = getVisibilityList(loc);
  } else {
    getSceneLines(loc, MaxRange, &visibilityList);
  }
  const vector<int>& locVisibilityList =
      (preRenderExists) ? (*preRenderVisibilityList) : visibilityList;

  for(size_t i=0; i<locVisibilityList.size(); i++){
    lines_p1x.push_back(lines[locVisibilityList[i]].P0().x);
    lines_p1y.push_back(lines[locVisibilityList[i]].P0().y);
    lines_p2x.push_back(lines[locVisibilityList[i]].P1().x);
    lines_p2y.push_back(lines[locVisibilityList[i]].P1().y);
    lines_color.push_back(0xFF00DE07);
  }
  vector<vector2f> rays;
  vector2f ray;
  bool duplicate;
  for(unsigned int i=0; i<scene_lines.size(); i++){
    lines_p1x.push_back(scene_lines[i].P0().x);
    lines_p1y.push_back(scene_lines[i].P0().y);
    lines_p2x.push_back(scene_lines[i].P1().x);
    lines_p2y.push_back(scene_lines[i].P1().y);
    lines_color.push_back(DebugColour);

    duplicate = false;
    ray = scene_lines[i].P0()-loc;
    for(size_t j=0; j<rays.size() && !duplicate; j++){
      if (ray.norm().dot(rays[j].norm()) > cos(RAD(0.1)) &&
          fabs(ray.length()-rays[j].length())<0.01) {
        duplicate = true;
        j = rays.erase(rays.begin()+j) - rays.begin();
      }
    }
    if(!duplicate)
      rays.push_back(ray);

    duplicate = false;
    ray = scene_lines[i].P1()-loc;
    for (size_t j=0; j<rays.size() && !duplicate; j++) {
      if (ray.norm().dot(rays[j].norm()) > cos(RAD(0.1)) &&
          fabs(ray.length()-rays[j].length())<0.01) {
        duplicate = true;
        j = rays.erase(rays.begin()+j) - rays.begin();
      }
    }
    if(!duplicate)
      rays.push_back(ray);
  }

  for(size_t i=0; i<rays.size(); i++){
    vector2f p0 = loc;
    vector2f p1 = rays[i]+loc;
    lines_p1x.push_back(p0.x);
    lines_p1y.push_back(p0.y);
    lines_p2x.push_back(p1.x);
    lines_p2y.push_back(p1.y);
    lines_color.push_back(LineExtentColor);
  }
}
