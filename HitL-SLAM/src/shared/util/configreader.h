#ifndef _INCLUDED_CONFIG_READER_H_
#define _INCLUDED_CONFIG_READER_H_

#include <vector>
#include <string>
#include <unistd.h>

#include "sstring.h"
#include "watch_files.h"


struct lua_State;

class ConfigReader{
public:
  enum FileFlags{
    Optional = 1<<0, // file need not be present for success
  };

  struct LuaTableEntry {
    enum LuaType {
      // @value points to a bool.
      kTypeBoolean = 0,
      // @value points to a double.
      kTypeNumber,
      // @value points to a std::string.
      kTypeString,
      // @value points to a LuaTableEntry.
      kTypeTable,
      // @value is NULL.
      kTypeOther
    };

    // The type of value of the table entry.
    LuaType type;

    // Technically LUA table keys can be of a number of types, but here we only
    // handle string keys that do not contain whitespaces in them.
    // TODO(joydeepb): add support for generic key values.
    std::string key;

    // Pointer to memory location that stores the entry value. The type of the
    // memory location pointed to depends on the entry type. See @LuaType for
    // more information on the C++ type used to represent each value type.
    void* value;

    LuaTableEntry() : type(kTypeOther), key(""), value(NULL) {}
    LuaTableEntry(LuaType _type, const std::string& _key, void* _value)
        : type(_type), key(_key), value(_value) {}
  };

  class SubTree{
    ConfigReader *config;
    CharString base,full;
    int errors_at_init;
  public:
    SubTree(ConfigReader &c,const char *base_exp);
    SubTree(SubTree      &s,const char *base_exp);

    int getErrors()
      {return(config->errors - errors_at_init);}

  protected:
    const char *getFullExp(const char *exp);
  public:
    const char *getStr(const char *exp,const char *default_val=NULL);
    bool getBool(const char *exp,bool   &val);
    bool getInt (const char *exp,int    &val);
    bool getUInt (const char *exp,unsigned int &val);
    bool getReal(const char *exp,float  &val);
    bool getReal(const char *exp,double &val);
    bool getPosReal(const char *exp,float  &val);
    bool getPosReal(const char *exp,double &val);
    bool getInt (const char *exp,int    &val,int    _min,int    _max);
    bool getReal(const char *exp,float  &val,float  _min,float  _max);
    bool getReal(const char *exp,double &val,double _min,double _max);

    template <class vec_t>
    bool getVec2f(const char *exp,vec_t &val);
    template <class vec_t>
    bool getVec3f(const char *exp,vec_t &val);
    template <class quaternion_t>
    bool getQuat4f(const char *exp,quaternion_t &val);
    template <class range_t>
    bool getRangeInt(const char *exp,range_t &val);
    template <class range_t>
    bool getRangeReal(const char *exp,range_t &val);
  };
protected:
  class FileHeader{
  public:
    CharString filename;     // name of file
    unsigned flags;          // flags from FileFlags
    WatchFiles::Watch watch; // file modification watch
  public:
    FileHeader()
      {}
    FileHeader(const FileHeader &fh);
  };

  std::vector<FileHeader> files; // list of config files to load
  lua_State *L;  // lua interpreter (valid between readFiles() and clear())
  int errors;    // count of errors from get* accesses
  int num_readfiles_calls;
  WatchFiles *watch_files; // class used to monitor files for changes
  bool modified; // true if config files need to be re-read
  char* path;
public:
  ConfigReader();
  ConfigReader(const char* _path);
  ~ConfigReader()
    {reset();}

protected:
  bool initLua();
  void closeLua();
  void clearWatches();
public:
  void close()
    {closeLua();}
  void clear()
    {closeLua(); errors = 0;}
  void reset();
  bool isOpen()
    {return(L != NULL);}

  void init()
    {watch_files=NULL; addStandard();}
  void init(WatchFiles &_watch_files)
    {watch_files=&_watch_files; addStandard();}
  void addFile(const char *filename,unsigned flags=0);

protected:
  void showError(int err_code,const char *filename);
  bool readFile(const char *filename,unsigned flags);
public:
  bool readInit()
    {return(initLua());}
  bool readFiles();
  bool isFileModified();

  /// get generation count in terms of # of readFiles() updates
  int getGeneration() const
    {return(num_readfiles_calls);}

  /// return true iff the client needs to get updated information
  /// (meaning it has been read since the last needUpdate call).  This
  /// call updates client_generation.
  bool needUpdate(int &client_generation) const;

  int getErrors()
    {return(errors);}

protected:
  void eval(const char *exp);
  void eval(const char *exp_base,const char *exp_ext);
public:
  const char *getStr(const char *exp,const char *default_val=NULL);
  bool getBool(const char *exp,bool   &val);
  bool getInt (const char *exp,int    &val);
  bool getUInt (const char *exp,unsigned int &val);
  bool getReal(const char *exp,float  &val);
  bool getReal(const char *exp,double &val);
  bool getPosReal(const char *exp,float &val);
  bool getPosReal(const char *exp,double &val);
  bool getInt (const char *exp,int    &val,int    _min,int    _max);
  bool getReal(const char *exp,float  &val,float  _min,float  _max);
  bool getReal(const char *exp,double &val,double _min,double _max);
  bool getTable(const char *exp, std::vector<LuaTableEntry>* entries);
  bool isTable(const char *exp);
  bool getTableEntry(const char *exp, LuaTableEntry* entry);
  bool getGlobalsList(std::vector<std::string>* globals);
  bool printGlobalsList();
  bool isValidSubtree(const char* sub_tree);
  bool set(const char *name,int val);
  bool set(const char *name,double val);

protected:
  void addStandard();
};

template <class vec_t>
bool ConfigReader::SubTree::getVec2f(const char *exp,vec_t &val)
{
  CharString fexp;

  fexp.printf("%s.x",exp);
  if(!getReal(fexp(),val.x)) return(false);

  fexp.printf("%s.y",exp);
  if(!getReal(fexp(),val.y)) return(false);

  return(true);
}

template <class vec_t>
bool ConfigReader::SubTree::getVec3f(const char *exp,vec_t &val)
{
  CharString fexp;

  fexp.printf("%s.x",exp);
  if(!getReal(fexp(),val.x)) return(false);

  fexp.printf("%s.y",exp);
  if(!getReal(fexp(),val.y)) return(false);

  fexp.printf("%s.z",exp);
  if(!getReal(fexp(),val.z)) return(false);

  return(true);
}

template <class quaternion_t>
bool ConfigReader::SubTree::getQuat4f(const char* exp, quaternion_t& val)
{
  CharString fexp;

  fexp.printf("%s.w",exp);
  if(!getReal(fexp(),val.w())) return(false);

  fexp.printf("%s.x",exp);
  if(!getReal(fexp(),val.x())) return(false);

  fexp.printf("%s.y",exp);
  if(!getReal(fexp(),val.y())) return(false);

  fexp.printf("%s.z",exp);
  if(!getReal(fexp(),val.z())) return(false);

  return(true);
}

template <class range_t>
bool ConfigReader::SubTree::getRangeInt(const char *exp,range_t &val)
{
  CharString fexp;

  fexp.printf("%s.min",exp);
  if(!getInt(fexp(),val.min)) return(false);

  fexp.printf("%s.max",exp);
  if(!getInt(fexp(),val.max)) return(false);

  return(true);
}

template <class range_t>
bool ConfigReader::SubTree::getRangeReal(const char *exp,range_t &val)
{
  CharString fexp;

  fexp.printf("%s.min",exp);
  if(!getReal(fexp(),val.min)) return(false);

  fexp.printf("%s.max",exp);
  if(!getReal(fexp(),val.max)) return(false);

  return(true);
}

#endif
