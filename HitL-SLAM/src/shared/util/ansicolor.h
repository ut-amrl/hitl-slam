#ifndef __ANSI_COLOR_H__
#define __ANSI_COLOR_H__

#include <stdio.h>

namespace AnsiColor{
  enum Color{
    Black   = 0,
    Red     = 1,
    Green   = 2,
    Yellow  = 3,
    Blue    = 4,
    Magenta = 5,
    Cyan    = 6,
    White   = 7,
    Default = 9,
  };

  static inline void emit(FILE *out,int code) {fprintf(out,"%c[%dm",27,code);}

  static inline void Reset(FILE *out) {emit(out,0);}

  static inline void Bold(FILE *out)      {emit(out,1);}
  static inline void Italics(FILE *out)   {emit(out,3);}
  static inline void Underline(FILE *out) {emit(out,4);}
  static inline void Inverse(FILE *out)   {emit(out,7);}
  static inline void Strike(FILE *out)    {emit(out,9);}

  static inline void BoldOff(FILE *out)      {emit(out,22);}
  static inline void ItalicsOff(FILE *out)   {emit(out,23);}
  static inline void UnderlineOff(FILE *out) {emit(out,24);}
  static inline void InverseOff(FILE *out)   {emit(out,27);}
  static inline void StrikeOff(FILE *out)    {emit(out,29);}

  static inline void SetFgColor(FILE *out,Color fg) {emit(out,30+fg);}
  static inline void SetBgColor(FILE *out,Color bg) {emit(out,40+bg);}
  static inline void SetColor(FILE *out,Color fg,Color bg)
    {emit(out,30+fg); emit(out,30+bg);}
};

#endif
