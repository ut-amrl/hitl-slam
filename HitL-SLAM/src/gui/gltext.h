//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    gltext.h
\brief   C++ Interface: GLText
\author  Joydeep Biswas (C) 2011
*/
//========================================================================

#include <eigen3/Eigen/Dense>
#include <GL/glu.h>
#include <QVector>
#include <QtGui>

#ifndef GL_TEXT_H
#define GL_TEXT_H

class GLText{
  struct Glyph{
    bool compiled;
    GLuint displayListID;
    float width;
    float height;
    float ascent;
    float descent;
    Glyph(){compiled=false;}
  };

  QVector<Glyph> glyphs;

  static const bool debugTesselation = false;
  QFont font;
  QFontMetricsF font_metrics;

public:

  typedef enum{
    LeftAligned,
    RightAligned,
    CenterAligned
  }HAlignOptions;

  typedef enum{
    TopAligned,
    BottomAligned,
    MedianAligned,
    MiddleAligned
  }VAlignOptions;

  GLText(QFont font = QFont(), float font_render_size = 1000.0);
  ~GLText();
  void drawString(
      const Eigen::Vector2f& loc, float angle, float size,
      const char* char_array,
      GLText::HAlignOptions hAlign=LeftAligned,
      GLText::VAlignOptions vAlign=MiddleAligned);
  void drawGlyph(QChar glyph);
  void initializeGlyph(QChar ch);
  float getWidth(QChar ch);
  float getHeight();
  Eigen::Vector2f getSize(QChar ch);
  float getWidth(const QString& str);
  float getAscent();
  float getDescent();

private:
  static const char* getPrimitiveType(GLenum type);
  static void tessBeginCB(GLenum which);
  static void tessEndCB();
  static void tessVertexCB(const GLvoid *data);
  static void tessErrorCB(GLenum errorCode);
  const float kFontRenderSize;
};

#endif //GL_TEXT_H