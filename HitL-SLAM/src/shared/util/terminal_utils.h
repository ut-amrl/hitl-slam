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
\file    terminal_utils.h
\brief   Subroutines to spice up stdout
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#ifndef TERMINAL_UTILS
#define TERMINAL_UTILS

#include <iostream>
#include <stdio.h>
#include <string>
//#include <printf.h>
//#include <stdarg.h>
//#include "geometry.h"



namespace TerminalUtils{
#ifdef WIN32
#include <windows.h>
static const unsigned char TERMINAL_ATTR_RESET     = 0;
static const unsigned char TERMINAL_ATTR_DIM       = 0;
static const unsigned char TERMINAL_ATTR_UNDERLINE = 0;
static const unsigned char TERMINAL_ATTR_BLINK     = 0;
static const unsigned char TERMINAL_ATTR_REVERSE   = 0;
static const unsigned char TERMINAL_ATTR_HIDDEN    = 0;

static const unsigned char TERMINAL_ATTR_BRIGHT    = 8;

static const unsigned char TERMINAL_COL_BLACK      = 0;
static const unsigned char TERMINAL_COL_BLUE       = 1;
static const unsigned char TERMINAL_COL_GREEN      = 2;
static const unsigned char TERMINAL_COL_CYAN       = 3;
static const unsigned char TERMINAL_COL_RED        = 4;
static const unsigned char TERMINAL_COL_MAGENTA    = 5;
static const unsigned char TERMINAL_COL_YELLOW     = 6;
static const unsigned char TERMINAL_COL_WHITE      = 7;

#else
static const unsigned char TERMINAL_ATTR_RESET     = 0;
static const unsigned char TERMINAL_ATTR_BRIGHT    = 1;
static const unsigned char TERMINAL_ATTR_DIM       = 2;
static const unsigned char TERMINAL_ATTR_UNDERLINE = 3;
static const unsigned char TERMINAL_ATTR_BLINK     = 4;
static const unsigned char TERMINAL_ATTR_REVERSE   = 7;
static const unsigned char TERMINAL_ATTR_HIDDEN    = 8;

static const unsigned char TERMINAL_COL_BLACK      = 0;
static const unsigned char TERMINAL_COL_RED        = 1;
static const unsigned char TERMINAL_COL_GREEN      = 2;
static const unsigned char TERMINAL_COL_YELLOW     = 3;
static const unsigned char TERMINAL_COL_BLUE       = 4;
static const unsigned char TERMINAL_COL_MAGENTA    = 5;
static const unsigned char TERMINAL_COL_CYAN       = 6;
static const unsigned char TERMINAL_COL_WHITE      = 7;
#endif
};

void ColourTerminal(unsigned char fg, unsigned char bg=TerminalUtils::TERMINAL_COL_BLACK, unsigned char attr=TerminalUtils::TERMINAL_ATTR_RESET);
void ResetTerminal();

void TerminalInformation(const char* text);
void TerminalAlert(const char* text);
void TerminalAlert(const std::string text);
void TerminalWarning(const char* text);
void TerminalWarning(const std::string text);

#endif //TERMINAL_UTILS
