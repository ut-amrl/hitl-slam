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
\file    vector_display.cpp
\brief   A GUI for Vector Localization; C++ Interface: VectorDisplay
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "vector_display.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <stdio.h>
#include <vector>

//#include "shared/util/helpers.h"
#include "eigen_helper.h"
#include "gltext.h"
#include "../shared/util/helpers.h"
#include "timer.h"

#define RAD(x) (M_PI / 180.0 * (x))
using Eigen::Rotation2Df;
using Eigen::Vector2f;
using std::max;
using std::min;
using std::size_t;
using std::string;
using std::vector;

const float VectorDisplay::minZValue = -10;
const float VectorDisplay::maxZValue = 10;

VectorDisplay::Color::Color(uint32_t col) {
  a = static_cast<float>((col & 0xFF000000ull)>>24)/255.0;
  r = static_cast<float>((col & 0xFF0000ull)>>16)/255.0;
  g = static_cast<float>((col & 0xFF00ull)>>8)/255.0;
  b = static_cast<float>(col & 0xFFull)/255.0;
}

VectorDisplay::Color::Color(float _r, float _g, float _b, float _a) {
  r = _r;
  g = _g;
  b = _b;
  a = _a;
}

VectorDisplay::Quad::Quad(const Vector2f &_p0, const Vector2f &_p1,
                          const Vector2f &_p2, const Vector2f &_p3) {
  p0 = _p0;
  p1 = _p1;
  p2 = _p2;
  p3 = _p3;
}

VectorDisplay::VectorDisplay(QWidget* parent) :
    QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
    gl_text(QFont()) {
  robotAngle = 0.0;
  robotLoc = Vector2f(0.0, 0.0);

  lineThickness = 1.5;
  pointsSize = 1.0;

  viewScale = 100.0 / static_cast<float>(min(width(), height()));
  viewXOffset = viewYOffset = 0.0;

  ptrMouseClickCallback = NULL;
  ptrKeyboardCallback = NULL;
  ptrMouseMoveCallback = NULL;
  followRobot = false;
  showRobot = true;
  rubberband_valid = false;
  enableDrawing = true;

  interaction_mode = kInteractionAuto;

  qRegisterMetaType<std::string>("std::string");
  connect(this, SIGNAL(RedrawSignal()), this, SLOT(RedrawSlot()));
  connect(this, SIGNAL(CaptureSignal(std::string)), this,
          SLOT(CaptureSlot(std::string)));
}

void VectorDisplay::EnableDrawing(bool enable) {
  printf("Enable drawing: %d\n", enable);
  this->setUpdatesEnabled(enable);
}

void VectorDisplay::RedrawSlot() {
  graphicsMutex.lock();
  if (enableDrawing) this->update();
  graphicsMutex.unlock();
}

void VectorDisplay::SetInteractionMode(VectorDisplay::InteractionMode mode) {
  interaction_mode = mode;
}

void VectorDisplay::mousePressEvent(QMouseEvent* event) {
  leftButton = event->buttons().testFlag(Qt::LeftButton);
  midButton = event->buttons().testFlag(Qt::MidButton);
  rightButton = event->buttons().testFlag(Qt::RightButton);
  bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  bool altKey = event->modifiers().testFlag(Qt::AltModifier);

  if (leftButton || midButton) {
    // Start Pan or Zoom
    mouseStartX = event->x();
    mouseStartY = event->y();
    RedrawSignal();
  }
  const float x = (static_cast<float>(event->x()) -
  0.5 * static_cast<float>(width())) * viewScale + viewXOffset;
  const float y = -(static_cast<float>(event->y()) -
  0.5 * static_cast<float>(height())) * viewScale + viewYOffset;
  mouse_down_loc = Vector2f(x, y);
  if (followRobot) mouse_down_loc = mouse_down_loc + robotLoc;
  mouseStartX = event->x();
  mouseStartY = event->y();
  if ((interaction_mode == kInteractionAuto &&
      (ctrlKey || shiftKey || altKey)) ||
      interaction_mode == kInteractionSelect) {
    const float x = (static_cast<float>(event->x()) -
        0.5 * static_cast<float>(width())) * viewScale + viewXOffset;
    const float y = -(static_cast<float>(event->y()) -
        0.5 * static_cast<float>(height())) * viewScale + viewYOffset;
    rubberband_valid = true;
    rubberband_start = Vector2f(x, y);
    rubberband_end = Vector2f(x, y);
    RedrawSignal();
  }
}

void VectorDisplay::mouseReleaseEvent(QMouseEvent* event) {
  bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  bool altKey = event->modifiers().testFlag(Qt::AltModifier);

  const float x = (static_cast<float>(event->x()) -
      0.5 * static_cast<float>(width())) * viewScale + viewXOffset;
  const float y = -(static_cast<float>(event->y()) -
      0.5 * static_cast<float>(height())) * viewScale + viewYOffset;
  Vector2f setLocation2(x, y);
  if (followRobot) setLocation2 = setLocation2 + robotLoc;
  const float setOrientation = atan2(setLocation2.y() - mouse_down_loc.y(),
                                      setLocation2.x() - mouse_down_loc.x());
  if (ptrMouseClickCallback) {
    const uint32_t modifier =
        (shiftKey ? 0x04 : 0) |
        (ctrlKey ? 0x02 : 0) |
        (altKey ? 0x01 : 0);
    ptrMouseClickCallback(mouse_down_loc, setLocation2, setOrientation, modifier);
  }
  if (rubberband_valid) {
    rubberband_valid = false;
    RedrawSignal();
  }
}

void VectorDisplay::mouseMoveEvent(QMouseEvent* event) {
  static const bool debug = false;
  bool leftButton = event->buttons().testFlag(Qt::LeftButton);
  bool midButton = event->buttons().testFlag(Qt::MidButton);
  bool rightButton = event->buttons().testFlag(Qt::RightButton);
  bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  bool altKey = event->modifiers().testFlag(Qt::AltModifier);

  if (debug) {
    printf("MouseMove Event (%d,%d), Left:%d Mid:%d Right:%d\n",
           event->x(), event->y(),
           leftButton?1:0, midButton?1:0, rightButton?1:0);
  }

  const float x = (static_cast<float>(event->x()) -
      0.5 * static_cast<float>(width())) * viewScale + viewXOffset;
  const float y = -(static_cast<float>(event->y()) -
      0.5 * static_cast<float>(height())) * viewScale + viewYOffset;

  if (rubberband_valid) {
    rubberband_end = Vector2f(x, y);
    RedrawSignal();
  }
  if (ptrMouseMoveCallback) {
    const uint32_t modifier =
        (shiftKey ? 0x04 : 0) |
        (ctrlKey ? 0x02 : 0) |
        (altKey ? 0x01 : 0);
    ptrMouseMoveCallback(Vector2f(x, y), event->buttons(), modifier);
  }
  if ((interaction_mode == kInteractionAuto &&
      (shiftKey || ctrlKey || altKey)) ||
      interaction_mode == kInteractionSelect) {
    // No need to do anything - location or target will be set
    return;
  }

  if ((interaction_mode == kInteractionAuto && leftButton) ||
      interaction_mode == kInteractionPan) {
    // Pan
    viewXOffset -= viewScale * static_cast<float>(event->x() - mouseStartX);
    viewYOffset += viewScale * static_cast<float>(event->y() - mouseStartY);
    mouseStartX = event->x();
    mouseStartY = event->y();
    setupViewport();
    RedrawSignal();
  } else if ((interaction_mode == kInteractionAuto && midButton) ||
      interaction_mode == kInteractionZoom) {
    // Zoom
    float zoomRatio = -static_cast<float>(event->y() - mouseStartY)/500.0;
    viewScale = viewScale/(1.0+zoomRatio);
    setupViewport();
    mouseStartX = event->x();
    mouseStartY = event->y();
    RedrawSignal();
  }
}

void VectorDisplay::Zoom(float zoom) {
  viewScale = viewScale/(1.0 + zoom);
  setupViewport();
  RedrawSignal();
}

void VectorDisplay::wheelEvent(QWheelEvent* event) {
  static const bool debug = false;
  float zoomRatio = static_cast<float>(event->delta())/1000.0;
  viewScale = viewScale/(1.0+zoomRatio);
  setupViewport();
  if (debug) printf("Zoom: %5.3f\n", viewScale);
  RedrawSignal();
}

bool VectorDisplay::CaptureSlot(const std::string& filename) {
  RedrawSlot();
  paintGL();
  QImage image = this->grabFrameBuffer();
  return image.save(filename.c_str());
}

void VectorDisplay::Capture(const std::string& filename) {
  CaptureSignal(filename);
}

template <typename T>
bool ReadElement(T* v, FILE* file) {
  return (fread(v, sizeof(T), 1, file) == 1);
}

template <typename T>
bool Read(Eigen::Matrix<T, 2, 1>* v, FILE* file) {
  if (ReadElement<T>(&v->x(), file) && ReadElement<T>(&v->y(), file)) {
    return true;
  }
  return false;
}

bool Read(VectorDisplay::Line* l, FILE* file) {
  if (Read(&l->p0, file) && Read(&l->p1, file)) return true;
  return false;
}

bool Read(VectorDisplay::Quad* q, FILE* file) {
  if (Read(&q->p0, file) &&
      Read(&q->p1, file) &&
      Read(&q->p2, file) &&
      Read(&q->p3, file)) return true;
  return false;
}

bool Read(VectorDisplay::Color* color, FILE* file) {
  if (ReadElement(&color->a, file) &&
      ReadElement(&color->r, file) &&
      ReadElement(&color->g, file) &&
      ReadElement(&color->b, file)) return true;
  return false;
}

template <typename T>
bool ReadArray(vector<T>* data_array, FILE* file) {
  int num_elements = 0;
  if (!ReadElement(&num_elements, file)) return false;
  data_array->resize(num_elements);
  for (auto i = 0; i < num_elements; ++i) {
    if (!Read(&((*data_array)[i]), file)) return false;
  }
  return true;
}

template <typename T>
void WriteElement(const T& v, FILE* file) {
  fwrite(&v, sizeof(T), 1, file);
}

template <typename T>
void Write(const Eigen::Matrix<T, 2, 1>& v, FILE* file) {
  WriteElement<T>(v.x(), file);
  WriteElement<T>(v.y(), file);
}

void Write(const VectorDisplay::Line& l, FILE* file) {
    Write(l.P0(), file);
    Write(l.P1(), file);
}

void Write(const VectorDisplay::Quad& q, FILE* file) {
  Write(q.P0(), file);
  Write(q.P1(), file);
  Write(q.P2(), file);
  Write(q.P3(), file);
}

void Write(const VectorDisplay::Color& color, FILE* file) {
  WriteElement(color.a, file);
  WriteElement(color.r, file);
  WriteElement(color.g, file);
  WriteElement(color.b, file);
}

template <typename T>
void WriteArray(const vector<T>& data_array, FILE* file) {
  const int array_length = data_array.size();
  WriteElement(array_length, file);
  for (auto&& element : data_array) {
    Write(element, file);
  }
}


void VectorDisplay::loadVector() {
  static const bool debug = false;
  const QString fileName = QFileDialog::getOpenFileName(
      this, QString("Load Vector"), QString(),
      QString("Vector Files (*.vec)"));
  if (debug) printf("Loading %s\n", fileName.toStdString().c_str());
  ScopedFile file(fileName.toStdString(), "r");
  graphicsMutex.lock();
  // Lines
  ReadArray(&lines, file);
  ReadArray(&lineColors, file);
  if (debug) printf("Loaded %d lines\n", static_cast<int>(lines.size()));

  // Points
  ReadArray(&points, file);
  ReadArray(&pointColors, file);
  if (debug) printf("Loaded %d points\n", static_cast<int>(points.size()));

  // Circles
  ReadArray(&circles, file);
  ReadArray(&circleColors, file);
  if (debug) printf("Loaded %d circles\n", static_cast<int>(circles.size()));

  graphicsMutex.unlock();
  RedrawSignal();
}

void VectorDisplay::saveVector() {
  static const bool debug = false;
  const QString fileName = QFileDialog::getSaveFileName(
      this, QString("Save Vector"), QString(),
      QString("Vector Files (*.vec)"));
  ScopedFile file(fileName.toStdString(), "w");
  graphicsMutex.lock();
  if (debug) printf("Saving  %s\n", fileName.toStdString().c_str());
  // Lines
  WriteArray(lines, file);
  WriteArray(lineColors, file);
  if (debug) printf("Saved %d lines\n", static_cast<int>(lines.size()));

  // Points
  WriteArray(points, file);
  WriteArray(pointColors, file);

  // Circles
  WriteArray(circles, file);
  WriteArray(circleColors, file);

  graphicsMutex.unlock();
}

void VectorDisplay::saveImage() {
  const QString fileName = QFileDialog::getSaveFileName(
      this, QString("Save Image"), QString(),
      QString("Image Files (*.png *.jpg *.bmp)"));
  Capture(fileName.toStdString());
}

void VectorDisplay::keyPressEvent(QKeyEvent* event) {
  static const bool debug = false;
  static const float kZoomRate = 1.01;
  static const float kPanRate = 4.0;
  bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  bool altKey = event->modifiers().testFlag(Qt::AltModifier);

  if (debug) printf("KeyPress: 0x%08X\n", event->key());
  switch (event->key()) {
    case Qt::Key_Q: {
      viewScale = viewScale * kZoomRate;
      setupViewport();
      RedrawSignal();
      break;
    }
    case Qt::Key_E: {
      viewScale = viewScale / kZoomRate;
      setupViewport();
      RedrawSignal();
      break;
    }
    case Qt::Key_W: {
      viewYOffset += kPanRate * viewScale;
      setupViewport();
      RedrawSignal();
      break;
    }
    case Qt::Key_P: {
      if (ctrlKey) {
        saveImage();
      }
      break;
    }
    case Qt::Key_S: {
      if (ctrlKey) {
        saveVector();
      } else {
        viewYOffset -= kPanRate * viewScale;
        setupViewport();
        RedrawSignal();
      }
      break;
    }
    case Qt::Key_O: {
      if (ctrlKey) {
        loadVector();
      }
      break;
    }
    case Qt::Key_D: {
      viewXOffset += kPanRate * viewScale;
      setupViewport();
      RedrawSignal();
      break;
    }
    case Qt::Key_A: {
      viewXOffset -= kPanRate * viewScale;
      setupViewport();
      RedrawSignal();
      break;
    }
    case Qt::Key_Space: {
      resetView();
      RedrawSignal();
      break;
    }
    case Qt::Key_Escape: {
      close();
      break;
    }
    case Qt::Key_F: {
      followRobot = !followRobot;
      if (followRobot) {
        viewXOffset = viewYOffset = 0.0;
      } else {
        viewXOffset += robotLoc.x();
        viewYOffset += robotLoc.y();
      }
      setupViewport();
      RedrawSignal();
      break;
    }
    case Qt::Key_R: {
      showRobot = !showRobot;
      RedrawSignal();
      break;
    }
    case Qt::Key_BracketLeft: {
      if (ctrlKey)
        pointsSize = max(0.1, pointsSize-0.1);
      else
        lineThickness = max(0.1, lineThickness-0.1);
      RedrawSignal();
      break;
    }
    case Qt::Key_BracketRight: {
      if (ctrlKey)
        pointsSize += 0.1;
      else
        lineThickness += 0.1;
      RedrawSignal();
      break;
    }
  }
  if (ptrKeyboardCallback) {
    const uint32_t modifier =
        (shiftKey ? 0x04 : 0) |
        (ctrlKey ? 0x02 : 0) |
        (altKey ? 0x01 : 0);
    ptrKeyboardCallback(event->key(), modifier);
  }
}

void VectorDisplay::SetPrimitivesSizes(float point_size, float line_thickness) {
  pointsSize = point_size;
  lineThickness = line_thickness;
}

void VectorDisplay::resetView() {
  viewScale = 100.0 / static_cast<float>(min(width(), height()));
  viewXOffset = viewYOffset = 0.0;
  setupViewport();
  RedrawSignal();
}

void VectorDisplay::initializeGL() {
  glEnable(GL_MULTISAMPLE);
}

void VectorDisplay::setupViewport() {
  const int window_width = width();
  const int window_height = height();
  glViewport(0, 0, window_width, window_height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-0.5*viewScale*window_width+viewXOffset,
          0.5*viewScale*window_width+viewXOffset,
          -0.5*viewScale*window_height+viewYOffset,
          0.5*viewScale*window_height+viewYOffset,
          minZValue, maxZValue);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void VectorDisplay::resizeGL(int width, int height) {
  setupViewport();
}

void VectorDisplay::drawCircles(float lineThickness) {
  static const float circleSize = 0.3;

  bool coloredCircles = (circleColors.size()== circles.size());
  if (coloredCircles) {
    for (int i =0; i < (int) circles.size(); i++) {
      glColor4f(circleColors[i].red(),
                circleColors[i].green(),
                circleColors[i].blue(), circleColors[i].alpha());
      drawArc(circles[i].x(), circles[i].y(),
              circleSize, circleSize + lineThickness, 0, 2.0 * M_PI);
    }
  } else {
    glColor4f(0.35, 0.35, 0.35, 1.0);
    for (int i =0; i < (int) circles.size(); i++) {
      drawArc(circles[i].x(), circles[i].y(),
              circleSize, circleSize + lineThickness, 0, 2.0 * M_PI);
    }
  }
}

void VectorDisplay::drawLine(
    const Vector2f& p0, const Vector2f& p1, float lineWidth) {
  const Vector2f line_dir = (p1 - p0).normalized();
  const Vector2f line_perp = Eigen::Perp2(line_dir);
  const Vector2f v0 = p0 + lineWidth * (-line_dir - line_perp);
  const Vector2f v1 = p1 + lineWidth * (line_dir - line_perp);
  const Vector2f v2 = p1 + lineWidth * (line_dir + line_perp);
  const Vector2f v3 = p0 + lineWidth * (-line_dir + line_perp);

  // drawQuad(v0, v1, v2, v3, 0.0);
  // glColor4f(0.35, 0.35, 0.35, 1.0);
  glBegin(GL_QUADS);
  glVertex3f(v0.x(), v0.y(), 0.0);
  glVertex3f(v1.x(), v1.y(), 0.0);
  glVertex3f(v2.x(), v2.y(), 0.0);
  glVertex3f(v3.x(), v3.y(), 0.0);
  glEnd();
}

void VectorDisplay::drawLine(const Line& line, float lineWidth) {
  drawLine(line.p0, line.p1, lineWidth);
}

void VectorDisplay::drawPoint(const Vector2f& loc, float pointSize) {
  drawQuad(Vector2f(loc.x()-pointSize, loc.y()-pointSize),
           Vector2f(loc.x()+pointSize, loc.y()-pointSize),
           Vector2f(loc.x()+pointSize, loc.y()+pointSize),
           Vector2f(loc.x()-pointSize, loc.y()+pointSize), 0.2);
}


void VectorDisplay::drawLines(float lineThickness) {
  bool coloredLines = (lineColors.size()== lines.size());
  if (coloredLines) {
    for (int i =0; i < (int) lines.size(); i++) {
      glColor4f(lineColors[i].red(),
                lineColors[i].green(),
                lineColors[i].blue(), lineColors[i].alpha());
      drawLine(lines[i], lineThickness);
    }
  } else {
    glColor4f(0.35, 0.35, 0.35, 1.0);
    for (int i =0; i < (int) lines.size(); i++) {
      drawLine(lines[i], lineThickness);
    }
  }
}

void VectorDisplay::drawPoints(float pointsSize) {
  bool coloredPoints = (pointColors.size()== points.size());
  if (coloredPoints) {
    for (int i =0; i < (int) points.size(); i++) {
      glColor4f(pointColors[i].red(),
                pointColors[i].green(),
                pointColors[i].blue(), pointColors[i].alpha());
      drawPoint(points[i], pointsSize);
    }
  } else {
    glColor4f(0.94, 0.46, 0.12, 1.0);

    for (int i =0; i < (int) points.size(); i++) {
      drawPoint(points[i], pointsSize);
    }
  }
}

void VectorDisplay::drawQuads() {
  bool coloredQuads = (quadColors.size()== quads.size());
  if (coloredQuads) {
    for (int i =0; i < (int) quads.size(); i++) {
      glColor4f(quadColors[i].red(), quadColors[i].green(),
                quadColors[i].blue(), quadColors[i].alpha());
      drawQuad(quads[i].P0(), quads[i].P1(), quads[i].P2(), quads[i].P3());
    }
  } else {
    glColor4f(0.94, 0.46, 0.12, 1.0);

    for (int i =0; i < (int) quads.size(); i++) {
      drawQuad(quads[i].P0(), quads[i].P1(), quads[i].P2(), quads[i].P3());
    }
  }
}

void VectorDisplay::drawTextStrings() {
  const bool use_colors = (text_strings.size() == text_colors.size());
  const bool valid_window_coords_flags =
      (text_strings.size() == text_in_window_coords.size());
  if (!use_colors) glColor4f(0.0, 0.0, 0.0, 1.0);
  for (size_t i = 0; i < text_strings.size(); ++i) {
    if (use_colors) {
      glColor4f(text_colors[i].red(), text_colors[i].green(),
                text_colors[i].blue(), text_colors[i].alpha());
    }
    if (valid_window_coords_flags && text_in_window_coords[i]) {
      // Translate to window coords.
      glPushMatrix();

      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      glOrtho(0, width(), -height(), 0, minZValue, maxZValue);
      glTranslatef(text_locations[i].x(), text_locations[i].y(), 0);
      gl_text.drawString(text_locations[i],
                        0,
                        text_heights[i],
                        text_strings[i].c_str(),
                        GLText::LeftAligned,
                        GLText::BottomAligned);

      glPopMatrix();
    } else {
      gl_text.drawString(text_locations[i],
                        0,
                        text_heights[i],
                        text_strings[i].c_str(),
                        GLText::LeftAligned,
                        GLText::BottomAligned);
    }
  }
}

void VectorDisplay::setView(float location_x, float location_y,
                            float window_size) {
  graphicsMutex.lock();
  viewXOffset = location_x;
  viewYOffset = location_y;
  viewScale = window_size / static_cast<float>(min(width(), height()));
  setupViewport();
  graphicsMutex.unlock();
  RedrawSignal();
}

void VectorDisplay::paintEvent(QPaintEvent* event) {
  // FunctionTimer ft(__FUNCTION__);
  graphicsMutex.lock();
  makeCurrent();
  glClearColor(BACKGROUND_COLOR);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  // glEnable(GL_DEPTH_TEST);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glEnable(GL_LINE_SMOOTH);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (false) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width(), -height(), 0, minZValue, maxZValue);
    glColor3f(0.0, 0.0, 0.0);
    string time_string;
    for (int i = 0; i < 20; ++i) {
      time_string += StringPrintf("%2d: %.3f\n", i, GetTimeSec());
    }
    gl_text.drawString(Vector2f(0, 0), 0, 20, time_string.c_str(),
      GLText::LeftAligned, GLText::TopAligned);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-0.5*viewScale*width()+viewXOffset,
            0.5*viewScale*width()+viewXOffset,
            -0.5*viewScale*height()+viewYOffset,
            0.5*viewScale*height()+viewYOffset,
            minZValue, maxZValue);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }
//   glMatrixMode(GL_MODELVIEW);
//   glPushMatrix();
//   glLoadIdentity();

  if (followRobot) {
    glLoadIdentity();
    glTranslatef(-(robotLoc.x()), -(robotLoc.y()), 0.0);
  }

  drawLines(lineThickness * viewScale);
  drawCircles(lineThickness * viewScale);
  drawPoints(pointsSize * viewScale);
  drawQuads();
  drawTextStrings();

  if (rubberband_valid) {
    glColor4f(0.0f, 0.0f, 0.0f, 1.0);
    drawLine(Line(rubberband_start, rubberband_end),
             static_cast<float>(lineThickness * viewScale));
  }

  // drawArc(0, 0, 1.0, 1.5, 0.0, 2.0 * M_PI);
  if (showRobot) {
    const float robotSize = 0.25;
    Vector2f h = Rotation2Df(robotAngle) * Vector2f(1.0, 0.0);
    Vector2f p = robotLoc + 2.0*robotSize*h;
    glColor4f(1.0, 0.416, 0.0, 1.0);
    drawArc(robotLoc, robotSize,
            robotSize+1.5*lineThickness * viewScale,
            0.0, 2.0 * M_PI, 0.0, RAD(4.0));
    drawLine(robotLoc, p, 0.75*lineThickness * viewScale);
  }

  swapBuffers();
  graphicsMutex.unlock();
}

void VectorDisplay::updateLines(
    const vector<Line>& _lines, const vector<Color>& _lineColors) {
  graphicsMutex.lock();
  lines = _lines;
  if (lines.size()== _lineColors.size())
    lineColors = _lineColors;
  else
    lineColors.clear();
  graphicsMutex.unlock();
  RedrawSignal();
}

void VectorDisplay::updatePoints(const vector<Vector2f>& _points,
                                 const vector<Color>& _pointColors) {
  graphicsMutex.lock();
  points = _points;
  if (points.size()== _pointColors.size())
    pointColors = _pointColors;
  else
    pointColors.clear();
  graphicsMutex.unlock();
  RedrawSignal();
}

void VectorDisplay::updateCircles(const vector<Vector2f>& _circles,
                                  const vector<Color>& _circleColors) {
  graphicsMutex.lock();
  circles = _circles;
  if (circles.size()== _circleColors.size())
    circleColors = _circleColors;
  else
    circleColors.clear();
  graphicsMutex.unlock();
  RedrawSignal();
}

void VectorDisplay::updateQuads(const vector<Quad>& _quads,
                                const vector<Color>& _quadColors) {
  graphicsMutex.lock();
  quads = _quads;
  if (quads.size()== _quadColors.size())
    quadColors = _quadColors;
  else
    quadColors.clear();
  graphicsMutex.unlock();
  RedrawSignal();
}

void VectorDisplay::updateText(const vector<Vector2f>& _text_locations,
                               const vector<string>& _text_strings,
                               const vector< float>& _text_heights,
                               const vector<Color>& _text_colors,
                               const vector<bool>& _text_in_window_coords) {
  graphicsMutex.lock();
  text_locations = _text_locations;
  text_strings = _text_strings;
  text_heights = _text_heights;
  text_colors = _text_colors;
  text_in_window_coords = _text_in_window_coords;
  graphicsMutex.unlock();
  RedrawSignal();
}

void VectorDisplay::updateDisplay(const Vector2f& _robotLoc,
                                  float _robotAngle,
                                  float _displayWindow,
                                  const vector<Line>& _lines,
                                  const vector<Vector2f>& _points,
                                  const vector<Vector2f>& _circles,
                                  const vector<Quad>& _quads,
                                  const vector<Color>& _lineColors,
                                  const vector<Color>& _pointColors,
                                  const vector<Color>& _circleColors,
                                  const vector<Color>& _quadColors,
                                  const vector<Vector2f>& _text_locations,
                                  const vector<string>& _text_strings,
                                  const vector<float>& _text_heights,
                                  const vector<Color>& _text_colors,
                                  const vector<bool>& _text_in_window_coords) {
  updateLines(_lines, _lineColors);
  updatePoints(_points, _pointColors);
  updateCircles(_circles, _circleColors);
  updateQuads(_quads, _quadColors);
  updateText(_text_locations,
             _text_strings,
             _text_heights,
             _text_colors,
             _text_in_window_coords);
  graphicsMutex.lock();
  robotAngle = _robotAngle;
  robotLoc = _robotLoc;
  graphicsMutex.unlock();
  RedrawSignal();
}

void VectorDisplay::resizeEvent(QResizeEvent* event) {
  QGLWidget::resizeEvent(event);
  RedrawSignal();
}


void VectorDisplay::drawQuad(
    const Vector2f& p0, const Vector2f& p1, const Vector2f& p2,
    const Vector2f& p3, float z) {
  glBegin(GL_QUADS);
  glVertex3f(p0.x(), p0.y(), z);
  glVertex3f(p1.x(), p1.y(), z);
  glVertex3f(p2.x(), p2.y(), z);
  glVertex3f(p3.x(), p3.y(), z);
  glEnd();
}

void VectorDisplay::drawArc(
    const Vector2f& loc, float r1, float r2, float theta1,
    float theta2, float z, float dTheta) {
  static const float tesselation = 0.1;
  if (dTheta < 0) {
    dTheta = tesselation/r2;
  }
  glBegin(GL_QUAD_STRIP);
  for (float theta = theta1; theta < theta2; theta+=dTheta) {
    float c1 = cos(theta), s1 = sin(theta);
    glVertex3d(r2*c1+loc.x(), r2*s1+loc.y(), z);
    glVertex3d(r1*c1+loc.x(), r1*s1+loc.y(), z);
  }
  float c1 = cos(theta2), s1 = sin(theta2);
  glVertex3d(r2*c1+loc.x(), r2*s1+loc.y(), z);
  glVertex3d(r1*c1+loc.x(), r1*s1+loc.y(), z);
  glEnd();
}

void VectorDisplay::ForceRedraw() {
  RedrawSignal();
}
