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
\file    vector_display.h
\brief   C++ Implementation: VectorDisplay
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <eigen3/Eigen/Dense>
#include <QWidget>
#include <QGLWidget>
#include <QMutex>
#include <QPainter>
#include <QPaintEvent>
#include <QFileDialog>
#include <vector>

#include "gltext.h"

#ifndef VECTOR_LIDAR_DISPLAY_H
#define VECTOR_LIDAR_DISPLAY_H


#define BACKGROUND_COLOR 1.0, 1.0, 1.0, 1.0

Q_DECLARE_METATYPE(std::string)

class VectorDisplay : public QGLWidget {
  Q_OBJECT

 public:
  struct Color {
    float r, g, b, a;
    Color() {r = g = b = 0.0; a = 1.0;}
    Color(float _r, float _g, float _b, float _a);
    explicit Color(uint32_t col);
    float red() const { return r; }
    float green() const { return g; }
    float blue() const { return b; }
    float alpha() const { return a; }
  };

  struct Line {
    Eigen::Vector2f p0, p1;
    Line(const Eigen::Vector2f &_p0, const Eigen::Vector2f &_p1) :
        p0(_p0), p1(_p1) {}
    Line() {}
    Eigen::Vector2f P0() const { return p0; }
    Eigen::Vector2f P1() const { return p1; }
  };

  struct Quad {
    Eigen::Vector2f p0, p1, p2, p3;
    Quad(const Eigen::Vector2f &_p0, const Eigen::Vector2f &_p1,
    const Eigen::Vector2f &_p2, const Eigen::Vector2f &_p3);
    Eigen::Vector2f P0() const { return p0; }
    Eigen::Vector2f P1() const { return p1; }
    Eigen::Vector2f P2() const { return p2; }
    Eigen::Vector2f P3() const { return p3; }
  };

  enum InteractionMode {
    kInteractionAuto = 0,
    kInteractionSelect,
    kInteractionPan,
    kInteractionZoom,
  };

  typedef void (*MouseClickCallback)
      (const Eigen::Vector2f&, const Eigen::Vector2f&, float, uint32_t);
  typedef void (*MouseMoveCallback)
      (const Eigen::Vector2f&, uint32_t, uint32_t);
  typedef void (*KeyboardCallback)(uint32_t, uint32_t);

  explicit VectorDisplay(QWidget *parent = 0);
  void updateLines(const std::vector<Line>& _lines,
                   const std::vector<Color>& _lineColors);
  void updatePoints(const std::vector<Eigen::Vector2f>& _points,
                    const std::vector<Color>& _pointColors);
  void updateCircles(const std::vector<Eigen::Vector2f>& _circles,
                     const std::vector<Color>& _circleColors);
  void updateQuads(const std::vector<Quad>& _quads,
                   const std::vector<Color>& _quadColors);
  void updateText(const std::vector<Eigen::Vector2f>& _text_locations,
                  const std::vector<std::string>& _text_strings,
                  const std::vector<float>& _text_heights,
                  const std::vector<Color>& _text_colors,
                  const std::vector<bool>& _text_in_window_coords);
  void updateDisplay(const Eigen::Vector2f& _robotLoc,
                     float _robotAngle,
                     float _displayWindow,
                     const std::vector<Line>& _lines,
                     const std::vector<Eigen::Vector2f>& _points,
                     const std::vector<Eigen::Vector2f>& _circles,
                     const std::vector<Quad>& _quads,
                     const std::vector<Color>& _lineColors,
                     const std::vector<Color>& _pointColors,
                     const std::vector<Color>& _circleColors,
                     const std::vector<Color>& _quadColors,
                     const std::vector<Eigen::Vector2f>& _text_locations,
                     const std::vector<std::string>& _text_strings,
                     const std::vector<float>& _text_heights,
                     const std::vector<Color>& _text_colors,
                     const std::vector<bool>& _text_in_window_coords);

  void resetView();
  void setView(float location_x, float location_y, float window_size);

  void ForceRedraw();

  // Set callback function to call when the display is clicked.
  // The first parameter of the callback is the click location (in world space),
  // the second the orientation (in radians), the third the click type
  void setMouseClickCallback(MouseClickCallback _ptrMouseClickCallback) {
    ptrMouseClickCallback = _ptrMouseClickCallback;
  }

  // Set callback function to call when a mouse move event occurs.
  void setMouseMoveCallback(MouseMoveCallback _ptrMouseMoveCallback) {
    ptrMouseMoveCallback = _ptrMouseMoveCallback;
  }

  // Set callback function to call when a keypress event occurs.
  // The first parameter is the keycode returned by QT.
  void setKeyboardCallback(KeyboardCallback _ptrKeyboardCallback) {
    ptrKeyboardCallback = _ptrKeyboardCallback;
  }

  // Set the current interaction mode.
  void SetInteractionMode(InteractionMode mode);

  // Capture the currecnt display contents and save it to the file specified.
  void Capture(const std::string& filename);

  // Zoom in / out.
  void Zoom(float zoom);

  // Enable / disable drawing.
  void EnableDrawing(bool enable);

  // Set the drawing size for lines and points.
  void SetPrimitivesSizes(float point_size, float line_width);

 private:
  static const float minZValue;
  static const float maxZValue;

  // Mutex to arbitrate access to drawing primitives and display
  QMutex graphicsMutex;
  std::vector<Line> lines;
  std::vector<Eigen::Vector2f> points;
  std::vector<Eigen::Vector2f> circles;
  std::vector<Quad> quads;
  std::vector<Color> lineColors;
  std::vector<Color> pointColors;
  std::vector<Color> circleColors;
  std::vector<Color> quadColors;
  std::vector<Eigen::Vector2f> text_locations;
  std::vector<std::string> text_strings;
  std::vector<float> text_heights;
  std::vector<Color> text_colors;
  std::vector<bool> text_in_window_coords;
  Eigen::Vector2f rubberband_start;
  Eigen::Vector2f rubberband_end;
  bool rubberband_valid;
  bool enableDrawing;

  float lineThickness;
  float pointsSize;

  Eigen::Vector2f robotLoc;
  float robotAngle;

  // The ratio of world-space coordinates to pixel space coordinates, in units
  // of meters / pixel.
  float viewScale;
  float viewXOffset;
  float viewYOffset;

  bool followRobot;
  bool showRobot;

  bool leftButton;
  bool midButton;
  bool rightButton;

  int mouseStartX;
  int mouseStartY;

  InteractionMode interaction_mode;

  // Callback for Mouse click events like setting location, GoTo point, etc.
  MouseClickCallback ptrMouseClickCallback;

  // Callback for Mouse move events.
  MouseMoveCallback ptrMouseMoveCallback;

  // Callback for Keyboard events.
  KeyboardCallback ptrKeyboardCallback;

  Eigen::Vector2f mouse_down_loc;

  // GLText class for drawing text.
  GLText gl_text;

  public slots:
  // Accept Signals to update display
  void RedrawSlot();
  bool CaptureSlot(const std::string& filename);

  signals:
  // Thread-safe way of scheduling display updates
  void RedrawSignal();
  void CaptureSignal(const std::string& filename);

 protected:
  void saveVector();
  void loadVector();
  void saveImage();
  void paintEvent(QPaintEvent * event);
  void wheelEvent(QWheelEvent * event);
  void mouseMoveEvent(QMouseEvent * event);
  void keyPressEvent(QKeyEvent * event);
  void mousePressEvent(QMouseEvent * event);
  void mouseReleaseEvent(QMouseEvent * event);
  void resizeEvent(QResizeEvent * event);
  void initializeGL();
  void resizeGL(int width, int height);
  void setupViewport();
  QSize sizeHint() const {return QSize(580, 1000);}

  void drawQuad(const Eigen::Vector2f& p0,
                const Eigen::Vector2f& p1,
                const Eigen::Vector2f& p2,
                const Eigen::Vector2f& p3,
                float z = -0.1);

  void drawArc(const Eigen::Vector2f& loc,
               float r1, float r2, float theta1, float theta2,
               float z = 0.0, float dTheta = -1);

  void drawArc(float x, float y, float r1, float r2,
               float theta1, float theta2, float z = 0.0,
               float dTheta = -1) {
    drawArc(Eigen::Vector2f(x, y), r1, r2, theta1, theta2, z, dTheta);
  }

  void drawCircles(float lineThickness);
  void drawLines(float lineThickness);
  void drawPoints(float pointsSize);
  void drawQuads();
  void drawTextStrings();
  void drawLine(const Line&line, float lineWidth);
  void drawLine(const Eigen::Vector2f& p0,
                const Eigen::Vector2f& p1,
                float lineWidth);
  void drawPoint(const Eigen::Vector2f& loc, float pointSize);
};

#endif  // VECTOR_LIDAR_DISPLAY_H
