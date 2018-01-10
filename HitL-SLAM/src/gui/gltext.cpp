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
\brief   C++ Implementation: GLText
\author  Joydeep Biswas (C) 2011
*/
//========================================================================

#include "gltext.h"

#include <eigen3/Eigen/Dense>
#include <QVector>
#include <QtGui>
#include <GL/glu.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <float.h>
// #include "util.h"

using Eigen::Vector2f;
using std::size_t;
using std::string;

GLText::GLText(QFont _font, float font_render_size) :
    font(_font), font_metrics(font), kFontRenderSize(font_render_size) {
  glyphs.clear();
  font.setPixelSize(kFontRenderSize);
  font_metrics = QFontMetricsF(font);
}

GLText::~GLText() {
}

void GLText::drawGlyph(QChar glyph) {
  const ushort& unicode_value = glyph.unicode();
  if(glyphs.size() < unicode_value + 1)
    glyphs.resize(unicode_value + 1);
  if(!glyphs[unicode_value].compiled)
    initializeGlyph(glyph);
  glCallList(glyphs[unicode_value].displayListID);
}

float GLText::getWidth(QChar ch) {
  const ushort& unicode_value = ch.unicode();
  if(glyphs.size() < unicode_value + 1)
    glyphs.resize(unicode_value + 1);
  if(!glyphs[unicode_value].compiled)
    initializeGlyph(ch);
  return glyphs[unicode_value].width;
}

float GLText::getHeight() {
  return (font_metrics.height() / kFontRenderSize);
}

Vector2f GLText::getSize(QChar ch)
{
  return Vector2f(getWidth(ch), getHeight());
}

float GLText::getWidth(const QString& str) {
  return (font_metrics.width(str) / kFontRenderSize);
}

float GLText::getAscent() {
  return (font_metrics.ascent() / kFontRenderSize);
}

float GLText::getDescent() {
  return (font_metrics.descent() / kFontRenderSize);
}

void GLText::drawString(
    const Vector2f& loc, float angle, float size, const char* char_array,
    HAlignOptions hAlign, VAlignOptions vAlign) {
  const QString str = QString::fromUtf8(char_array);
  if (str.length() < 1) return;
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslated(loc.x(),loc.y(),0.0);
  glScaled(size, size,1.0);
  glRotated(angle,0,0,1);

  switch(hAlign){
    case LeftAligned:{
      // Normal rendering will achieve this.
      break;
    }
    case RightAligned:{
      glTranslated(-getWidth(str),0,0);
      break;
    }
    case CenterAligned:{
      glTranslated(-0.5 * getWidth(str),0,0);
      break;
    }
  }
  switch(vAlign){
    case BottomAligned:{
      glTranslated(0.0,getDescent(),0.0);
      break;
    }
    case TopAligned:{
      glTranslated(0.0,-getAscent(),0.0);
      break;
    }
    case MedianAligned:{
      //Normal rendering will achieve this!
      break;
    }
    case MiddleAligned:{
      glTranslated(0.0,-0.5*getHeight(),0.0);
      break;
    }
  }
  static const QChar kLineFeed(static_cast<char>('\n'));
  static const QChar kCarriageReturn(static_cast<char>('\r'));
  static const QChar kTabStop(static_cast<char>('\t'));
  static const QChar kSpace(static_cast<char>(' '));
  const float tab_width = 2.0 * getWidth(kSpace);
  float line_length = 0.0;
  for (int i = 0; i < str.length(); ++i) {
    const QChar& letter = str.at(i);
    if (letter == kLineFeed || letter == kCarriageReturn) {
      glTranslated(-line_length, -getHeight(), 0.0);
      line_length = 0.0;
      continue;
    }
    if (letter == kTabStop) {
      glTranslated(tab_width, 0.0, 0.0);
      line_length += tab_width;
      continue;
    }
    drawGlyph(letter);
    const float d = getWidth(letter);
    line_length += d;
    glTranslated(d, 0.0, 0.0);
    /*
    if (i < str.length() - 1) {
      d += 0.5 * getWidth(str[i + 1]);
      glTranslated(d, 0.0, 0.0);
    }
    */
  }
  glPopMatrix();
}

void GLText::initializeGlyph(QChar ch) {
  static const bool debug = false;
  Glyph glyph;
  const QString q_string(ch);

  QPainterPath path;
  path.addText(0,0,font, q_string);
  QList<QPolygonF> polygons = path.toSubpathPolygons();
  glyph.ascent = font_metrics.ascent() / kFontRenderSize;
  glyph.descent = font_metrics.descent() / kFontRenderSize;

  glyph.height = font_metrics.height() / kFontRenderSize;
  glyph.width = font_metrics.width(q_string) / kFontRenderSize;


  int numVertices = 0;
  for(int i=0; i<polygons.size(); i++){
    numVertices += polygons[i].size();
  }
  GLdouble vertices[numVertices][3];
  int j=0;
  for(int i=0; i<polygons.size(); i++){
    for(int k=0; k<polygons[i].size(); k++){
      vertices[j][0] = polygons[i][k].x()/kFontRenderSize;
      vertices[j][1] = -polygons[i][k].y()/kFontRenderSize;
      vertices[j][2] = 9;
      j++;
    }
  }

  if (debug) {
    printf("Glyph for QChar 0x%04hX, '%c' Width %f, %d polygons, %d vertices\n",
           ch.unicode(), ch.toAscii(), glyph.width, polygons.size(),
           numVertices);
  }
  GLUtesselator* tess = gluNewTess();
  gluTessCallback(tess, GLU_TESS_BEGIN, (_GLUfuncptr) tessBeginCB);
  gluTessCallback(tess, GLU_TESS_END, (_GLUfuncptr) tessEndCB);
  gluTessCallback(tess, GLU_TESS_ERROR, (_GLUfuncptr) tessErrorCB);
  gluTessCallback(tess, GLU_TESS_VERTEX, (_GLUfuncptr) tessVertexCB);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glyph.displayListID = glGenLists(1);
  if(glyph.displayListID==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(glyph.displayListID, GL_COMPILE);
  gluTessBeginPolygon(tess, 0);
  j=0;
  for(int i=0; i<polygons.size(); i++){
    gluTessBeginContour(tess);
    for(int k=0; k<polygons[i].size(); k++){
      gluTessVertex(tess, vertices[j], vertices[j]);
      j++;
    }
    gluTessEndContour(tess);
  }
  gluTessEndPolygon(tess);
  gluDeleteTess(tess);
  glEndList();
  glPopMatrix();
  glyph.compiled = true;
  glyphs[ch.unicode()] = glyph;
}

const char* GLText::getPrimitiveType(GLenum type)
{
  switch(type)
  {
    case 0x0000:
      return "GL_POINTS";
      break;
    case 0x0001:
      return "GL_LINES";
      break;
    case 0x0002:
      return "GL_LINE_LOOP";
      break;
    case 0x0003:
      return "GL_LINE_STRIP";
      break;
    case 0x0004:
      return "GL_TRIANGLES";
      break;
    case 0x0005:
      return "GL_TRIANGLE_STRIP";
      break;
    case 0x0006:
      return "GL_TRIANGLE_FAN";
      break;
    case 0x0007:
      return "GL_QUADS";
      break;
    case 0x0008:
      return "GL_QUAD_STRIP";
      break;
    case 0x0009:
      return "GL_POLYGON";
      break;
    default:
      return "UNKNOWN_PRIMITIVE";
      break;
  }
}

void GLText::tessBeginCB(GLenum which)
{
  glBegin(which);
  if(debugTesselation) printf("glBegin(%s);\n",getPrimitiveType(which));
}

void GLText::tessEndCB()
{
  glEnd();
  if(debugTesselation) printf("glEnd();\n");
}

void GLText::tessVertexCB(const GLvoid *data)
{
  const GLdouble *ptr = (const GLdouble*)data;
  if(debugTesselation) printf("glVertex3d(%f,%f,%f);\n",*ptr,*(ptr+1),*(ptr+2));
  glVertex3dv(ptr);
}

void GLText::tessErrorCB(GLenum errorCode)
{
  const GLubyte *errorStr;
  errorStr = gluErrorString(errorCode);
  printf("[ERROR]: %s\n", errorStr);
}
