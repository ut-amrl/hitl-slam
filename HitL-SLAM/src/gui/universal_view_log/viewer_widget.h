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
\file    viewer_widget.h
\brief   C++ Interface: ViewerWidget
\author  Joydeep Biswas, (C) 2010

ViewerWidget is the base class used to design QT widgets for the universal 
logging / viewer application

*/
//========================================================================

#ifndef VIEWER_WIDGET_H
#define VIEWER_WIDGET_H

#include <QtGui/QWidget>
#include <google/protobuf/message.h>

using namespace google::protobuf;

class ViewerWidget : public QWidget 
{
  Q_OBJECT
  
public:
  ViewerWidget( QWidget * parent = 0, Qt::WindowFlags  f = 0 );
  ~ViewerWidget();
  
  /// The update function should be overridden by subclasses in order to receive update messages
  virtual bool update(const Message& msg){return false;}
};

#endif //VIEWER_WIDGET_H