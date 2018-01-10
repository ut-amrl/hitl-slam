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
\file    py_vector_display.h
\brief   A python interface for the Vector Display
\author  Michael Murphy
 */
//========================================================================

#ifndef PY_VECTOR_DISPLAY_H
#define PY_VECTOR_DISPLAY_H

#include <QWidget>

namespace ros {
  class NodeHandle;
}  // namespace ros

class PyVectorDisplay {
public:
  static QWidget* initializeForPythonUI();
  static void AutoLocalize(double x, double y, double theta);
  static void SetInteractionPan();
  static void SetInteractionZoom();
  static void SetInteractionGoto();
  static void SetInteractionLocalize();
  static void SetInteractionClick();
  static void ZoomIn();
  static void ZoomOut();
  static void ChangeMap();
  static void shutdown();
  static void EnableDrawing();
  static void DisableDrawing();
private:
  static ros::NodeHandle* node_handle_;
};

#endif  // PY_VECTOR_DISPLAY_H
