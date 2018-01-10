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
\file    terminal_utils.cpp
\brief   Subroutines to spice up stdout
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "terminal_utils.h"

void ColourTerminal(unsigned char fg, unsigned char bg, unsigned char attr)
{
#ifdef WIN32
  HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
  SetConsoleTextAttribute(hConsole, fg | (bg<<4) | attr);
#else
  //printf("%c[%d;%d;%dm", 0x1B, attr, fg + 30, bg + 40);
  printf("\033[%d;%dm", attr, fg + 30);
  fflush(stdout);
#endif
}

void ResetTerminal()
{
#ifdef WIN32
  ColourTerminal(TerminalUtils::TERMINAL_COL_WHITE);
#else
  printf("\033[0m");
  fflush(stdout);
#endif
}

void TerminalInformation(const char* text)
{
  ColourTerminal(TerminalUtils::TERMINAL_COL_BLUE, TerminalUtils::TERMINAL_COL_BLACK, TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("%s\n",text);
  ResetTerminal();
}

void TerminalAlert(const char* text)
{
  ColourTerminal(TerminalUtils::TERMINAL_COL_GREEN,
                 TerminalUtils::TERMINAL_COL_BLACK,
                 TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("%s\n",text);
  ResetTerminal();
}

void TerminalAlert(const std::string text)
{
  ColourTerminal(TerminalUtils::TERMINAL_COL_GREEN,
                 TerminalUtils::TERMINAL_COL_BLACK,
                 TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("%s\n", text.c_str());
  ResetTerminal();
}

void TerminalWarning(const char* text)
{
  ColourTerminal(TerminalUtils::TERMINAL_COL_RED,
                 TerminalUtils::TERMINAL_COL_BLACK,
                 TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("%s\n",text);
  ResetTerminal();
}

void TerminalWarning(const std::string text)
{
  ColourTerminal(TerminalUtils::TERMINAL_COL_RED,
                 TerminalUtils::TERMINAL_COL_BLACK,
                 TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("%s\n",text.c_str());
  ResetTerminal();
}
