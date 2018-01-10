// (C) 2004-2005 James R. Bruce, Carnegie Mellon University
// Licenced under the GNU General Public License (GPL) version 2,
//   or alternately by a specific written agreement.

#ifndef __SIMPLE_STRING_H__
#define __SIMPLE_STRING_H__

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define SSTRING_TEM template <class char_t>
#define SSTRING_FUN SimpleString<char_t>

SSTRING_TEM
class SimpleString{
  char_t *buf; //!< storage buffer for the null terminated string
  int size; //!< size of the storage buffer (not string length)

public:
  static const int LocNone = -1;

public:
  // creation and assignment calls
  SimpleString()
    {buf=NULL; size=0;}
  SimpleString(const char *str)
    {buf=NULL; size=0; copy(str);}
  SimpleString(const char *str,int length)
    {buf=NULL; size=0; copy(str,length);}
  SimpleString(const SimpleString &str)
    {buf=NULL; size=0; copy(str.buf);}
  SimpleString(int expected_size)
    {buf=NULL; size=0; allocate(expected_size);}
  ~SimpleString()
    {reset();}
  void reset();
  void clear()
    {reset();}

  SimpleString &operator=(const SimpleString &str)
    {copy(str.buf); return(*this);}
  SimpleString &operator=(const char *str)
    {copy(str); return(*this);}

protected:
  bool allocate(int _size);
  bool reallocate(int _size);
public:
  bool valid()
    {return(buf != NULL);}

  bool copy(const char *str);
  bool copy(const char *str,int length);
  bool copy(const SimpleString &str)
    {return(copy(str.buf));}
  bool copy(const SimpleString &str,int start);
  bool copy(const SimpleString &str,int start,int end);
  bool remove(int start,int end);
  void truncate(int len);

  bool add(const char *str);
  bool add(const SimpleString &str)
    {return(add(str.buf));}
  SimpleString &operator+=(const char *str)
    {add(str); return(*this);}
  SimpleString &operator+=(const SimpleString &str)
    {add(str); return(*this);}

  // string accessors
  const char_t *operator()() const
    {return(buf);}
  int length() const
    {return(strlen(buf));}
  char_t operator[](int i) const
    {return(buf[i]);}

  // comparison and search
  bool operator==(const SimpleString &str) const
    {return(strcmp(buf,str.buf) == 0);}
  bool operator==(const char *cstr) const
    {return(strcmp(buf,cstr) == 0);}
  bool operator!=(const SimpleString &str) const
    {return(strcmp(buf,str.buf) != 0);}
  bool operator!=(const char *cstr) const
    {return(strcmp(buf,cstr) != 0);}

  bool begins(const char *str) const
    {return(memcmp(buf,str,strlen(str)) == 0);}

  int findNext(char_t key,int col) const;
  int findFirst(char_t key) const
    {return(findNext(key,0));}
  int findLast(char_t key) const;

  bool getInt   (int &col,int    &val);
  bool getFloat (int &col,float  &val);
  bool getDouble(int &col,double &val);

  // formatted printing into string
  int printf(const char *format, ...)
    __attribute__((format(__printf__,2,3)));
  // file input
  bool fgets(FILE *in,int maxsize);
};

SSTRING_TEM
void SSTRING_FUN::reset()
{
  delete[](buf);
  buf = NULL;
  size = 0;
}

// make sure we have sufficient storage
SSTRING_TEM
bool SSTRING_FUN::allocate(int _size)
{
  if(buf!=NULL && _size<size) return(true);

  delete[](buf);
  buf = new char_t[_size];

  if(buf){
    size = _size;
    buf[0] = 0;
  }else{
    size = 0;
  }

  return(buf != NULL);
}

SSTRING_TEM
bool SSTRING_FUN::reallocate(int _size)
{
  if(buf!=NULL && _size<size) return(true);

  char_t *nbuf = new char_t[_size];

  if(nbuf){
    int ms = (size < _size)? size : _size;
    memcpy(nbuf,buf,ms);
    size = _size;
  }else{
    size = 0;
  }

  delete[](buf);
  buf = nbuf;

  return(buf != NULL);
}

SSTRING_TEM
bool SSTRING_FUN::copy(const char *str)
{
  int sz = strlen(str)+1;
  if(!allocate(sz)) return(false);
  memcpy(buf,str,sz*sizeof(char_t));
  return(true);
}

SSTRING_TEM
bool SSTRING_FUN::copy(const char *str,int length)
{
  int sz = 0;
  while(str[sz] && sz<length) sz++;
  if(!allocate(sz+1)) return(false);
  memcpy(buf,str,sz*sizeof(char_t));
  buf[sz]=0;
  return(true);
}

SSTRING_TEM
bool SSTRING_FUN::copy(const SimpleString &str,int start)
{
  int l = str.length();
  if(start<0 || start>l) return(false);
  int sz = l-start+1;
  allocate(sz);
  memcpy(buf,str.buf+start,sz*sizeof(char_t));
  return(true);
}

SSTRING_TEM
bool SSTRING_FUN::copy(const SimpleString &str,int start,int end)
{
  int l = str.length();
  if(start>end || start<0 || end>l) return(false);
  int sz = end-start+1;
  allocate(sz);
  memcpy(buf,str.buf+start,sz*sizeof(char_t));
  return(true);
}

SSTRING_TEM
bool SSTRING_FUN::remove(int start,int end)
{
  int l = length();
  if(start>end || start<0 || end>l) return(false);
  if(start==end) return(true);
  int sz = l-end+1;
  memmove(buf+start,buf+end,sz*sizeof(char_t));
  return(true);
}

SSTRING_TEM
void SSTRING_FUN::truncate(int len)
{
  int l = length();
  if(len>=0 && len<l) buf[len] = 0;
}

SSTRING_TEM
bool SSTRING_FUN::add(const char *str)
{
  int base = length();
  int sz   = strlen(str)+1;
  if(!reallocate(base+sz)) return(false);
  memcpy(buf+base,str,sz*sizeof(char_t));
  return(true);
}

SSTRING_TEM
int SSTRING_FUN::findNext(char_t key,int col) const
{
  char_t ch;

  while((ch=buf[col])!=0 && ch!=key) col++;
  return((ch == key)? col : LocNone);
}

SSTRING_TEM
int SSTRING_FUN::findLast(char_t key) const
{
  int i = 0, k = LocNone;
  char_t ch;

  while((ch = buf[i]) != 0){
    if(ch == key) k = i;
    i++;
  }

  return(k);
}

SSTRING_TEM
bool SSTRING_FUN::getInt(int &col,int &val)
{
  char *start = buf + col,*end;
  int v = strtol(start,&end,10);
  col = end - buf;
  bool ok = (end != start);
  if(ok) val = v;
  return(ok);
}

SSTRING_TEM
bool SSTRING_FUN::getFloat(int &col,float &val)
{
  char *start = buf + col,*end;
  float v = strtof(start,&end);
  col = end - buf;
  bool ok = (end != start);
  if(ok) val = v;
  return(ok);
}

SSTRING_TEM
bool SSTRING_FUN::getDouble(int &col,double &val)
{
  char *start = buf + col,*end;
  double v = strtod(start,&end);
  col = end - buf;
  bool ok = (end != start);
  if(ok) val = v;
  return(ok);
}

SSTRING_TEM
int SSTRING_FUN::printf(const char *format, ...)
{
  va_list al;
  int r = 0;

  va_start(al,format);
  r = vsnprintf(buf,size,format,al);
  va_end(al);
  if(r < size) return(r);

  if(!allocate(r+1)) return(LocNone);

  va_start(al,format);
  r = vsnprintf(buf,size,format,al);
  va_end(al);
  return(r);
}

SSTRING_TEM
bool SSTRING_FUN::fgets(FILE *in,int maxsize)
{
  if(!allocate(maxsize) || maxsize==0) return(false);
  buf[0] = 0;
  return(::fgets(buf,size,in) != NULL);
}

typedef SimpleString<char> CharString;
typedef SimpleString<unsigned char> UCharString;

class CharStringLessThan{
public:
  bool operator()(const CharString &a,const CharString &b) const
    {return(strcmp(a(),b()) < 0);}
};

#endif
