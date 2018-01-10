#include <sys/types.h>
#include <sys/stat.h>

#include <cmath>
#include <stdio.h>

extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
};

#include "ansicolor.h"
#include "util.h"

#include "configreader.h"


static const char OverrideLuaGlobalsCommand[] = " \
local newgt = {} \n \
setmetatable(newgt, {__index = _G}) \n \
setfenv(0, newgt) \n \
";

const char LUAGetTableEntries[] = "\
function _table_entries(_table) \n \
  local str = \"\"; \n \
  for k,v in pairs(_table) do \n \
    str = str .. k .. \";\" ; \n \
  end \n \
  return str; \n \
end \n \
";

static const bool Debug = false;

using std::string;
using std::vector;

namespace {
#define CHECK_NOTNULL(x) \
  if (x==NULL) { \
    printf(#x " is NULL @%s:%d\n", __FILE__, __LINE__); \
    exit(1); \
  }

void SplitKeys(const char* keys, vector<string>* keys_vector) {
  CHECK_NOTNULL(keys_vector);
  char* key_name = strtok(const_cast<char*>(keys), ";");
  while (key_name != NULL) {
    if (key_name[0] != '_' && strlen(key_name) > 0) {
      keys_vector->push_back(string(key_name));
    }
    key_name = strtok(NULL, ";");
  }
}

}  // namespace

/*
  [ References ]

  Lua for configuration files:
    Programming in  Lua
    Chapter 25. Extending your Application
    http://www.lua.org/pil/25.html
  Lua C API function documentation:
    Lua 5.0 Reference Manual
    Chapter 3 - The Application Program Interface
    http://www.lua.org/manual/5.0/manual.html#3
*/

//====================================================================//

bool FileExists(const char *filename)
{
  struct stat st;
  return(stat(filename,&st) == 0);
}

//====================================================================//

bool ConfigReader::isValidSubtree(const char* sub_tree)
{
  eval(sub_tree);
  bool ok = lua_istable(L, -1);
  if(!ok){
    printf("ConfigReader: \"%s\" is not a valid sub-tree.\n",sub_tree);
    errors++;
  }
  lua_pop(L,-1);
  return(ok);
}

ConfigReader::SubTree::SubTree(ConfigReader &c,const char *base_exp)
{
  config = &c;
  base = base_exp;
  errors_at_init = config->errors || !config->isValidSubtree(base());
}

ConfigReader::SubTree::SubTree(SubTree &s,const char *base_exp)
{
  config = s.config;
  const char *sep = (base_exp[0]=='[')? "" : ".";
  base.printf("%s%s%s",s.base(),sep,base_exp);
  errors_at_init = config->errors || !config->isValidSubtree(base());
}

const char *ConfigReader::SubTree::getFullExp(const char *exp)
{
  const char *sep = (exp[0]=='[')? "" : ".";
  full.printf("%s%s%s",base(),sep,exp);
  return(full());
}

const char *ConfigReader::SubTree::getStr(const char *exp,
                                          const char *default_val)
{
  return(config->getStr(getFullExp(exp),default_val));
}

bool ConfigReader::SubTree::getBool(const char *exp,bool &val)
{
  if (errors_at_init) return false;
  return(config->getBool(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getInt(const char *exp,int &val)
{
  if (errors_at_init) return false;
  return(config->getInt(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getUInt(const char* exp, unsigned int& val)
{
  if (errors_at_init) return false;
  return(config->getUInt(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getReal(const char *exp,float  &val)
{
  if (errors_at_init) return false;
  return(config->getReal(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getReal(const char *exp,double &val)
{
  if (errors_at_init) return false;
  return(config->getReal(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getPosReal(const char *exp,float &val)
{
  if (errors_at_init) return false;
  return(config->getPosReal(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getPosReal(const char *exp,double &val)
{
  if (errors_at_init) return false;
  return(config->getPosReal(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getInt(const char *exp,int &val,
                                   int _min,int _max)
{
  if (errors_at_init) return false;
  return(config->getInt(getFullExp(exp),val,_min,_max));
}

bool ConfigReader::SubTree::getReal(const char *exp,float &val,
                                    float _min,float _max)
{
  if (errors_at_init) return false;
  return(config->getReal(getFullExp(exp),val,_min,_max));
}

bool ConfigReader::SubTree::getReal(const char *exp,double &val,
                                    double _min,double _max)
{
  if (errors_at_init) return false;
  return(config->getReal(getFullExp(exp),val,_min,_max));
}

ConfigReader::FileHeader::FileHeader(const FileHeader &fh)
{
  filename = fh.filename;
  flags = fh.flags;
  watch = fh.watch;
}

//====================================================================//

ConfigReader::ConfigReader()
{
  path = NULL;
  L = NULL;
  errors = 0;
  num_readfiles_calls = 0;
  watch_files = NULL;
  modified = false;
}

ConfigReader::ConfigReader(const char* _path)
{
  path = (char*) malloc(strlen(_path)+1);
  strcpy(path,_path);
  L = NULL;
  errors = 0;
  num_readfiles_calls = 0;
  watch_files = NULL;
  modified = false;
}

bool ConfigReader::initLua()
{
  if(L) closeLua();
  L = lua_open();
  if(!L) return(false);

  // load libraries
  luaL_openlibs(L);

  // discard any results from initialization
  lua_settop(L, 0);

  return(true);
}

void ConfigReader::closeLua()
{
  if(L) lua_close(L);
  L = NULL;
}

void ConfigReader::clearWatches()
{
  for(unsigned i=0; i<files.size(); i++){
    files[i].watch.remove();
  }
}

void ConfigReader::reset()
{
  closeLua();
  clearWatches();
  files.clear();
  num_readfiles_calls = 0;
  watch_files = NULL;
}

void ConfigReader::addFile(const char *filename,unsigned flags)
{
  FileHeader fh;
  if(path!=NULL){
    fh.filename = path;
    fh.filename.add(filename);
  }else{
    fh.filename = filename;
  }
  fh.flags = flags;
  fh.watch.watch(watch_files,filename);
  files.push_back(fh);

  modified = true;
}

void ConfigReader::showError(int err_code,const char *filename)
{
  if(err_code == 0) return;

  AnsiColor::SetFgColor(stderr,AnsiColor::Yellow);
  AnsiColor::Bold(stderr);

  const char *err_str;
  switch(err_code){
    case LUA_ERRFILE:   err_str="File not found"; break;
    case LUA_ERRSYNTAX: err_str="Syntax error"; break;
    case LUA_ERRMEM:    err_str="Memory allocation error"; break;
    case LUA_ERRRUN:    err_str="Runtime error"; break;
    case LUA_ERRERR:    err_str="Error running error handler"; break;
    default: err_str="Uknown error";
  }

  int t = lua_gettop(L);
  if(lua_isstring(L,t)){
    const char *str = lua_tolstring(L,t,NULL);
    fprintf(stderr,"ConfigReader: %s\n",str);
  }

  fprintf(stderr,"ConfigReader: %s: %s\n",filename,err_str);

  AnsiColor::Reset(stderr);
}


bool ConfigReader::readFile(const char *filename,unsigned flags)
{
  if(Debug){
    printf("ConfigReader: reading \"%s\"\n",filename);
  }

  // it's ok if optional files don't exist
  if((flags & Optional)!=0 && !FileExists(filename)) return(true);

  // try to load the file
  int ret = luaL_loadfile(L, filename);
  if(ret){
    showError(ret,filename);
  }else{
    // try to execute the file
    ret = lua_pcall(L, 0, LUA_MULTRET, 0);
    if(ret) showError(ret,filename);
  }

  lua_settop(L, 0); // discard any results

  return(ret == 0);
}

bool ConfigReader::readFiles()
{
  if(!L && !initLua()) return(false);
  errors = 0;
  modified = false;
  bool ok = true;

  luaL_dostring(L,OverrideLuaGlobalsCommand);
  luaL_dostring(L,LUAGetTableEntries);

  for(unsigned i=0; i<files.size(); i++){
    FileHeader &fh = files[i];
    if(fh.watch.isFileModified()){
      fh.watch.rewatch(fh.filename());
    }
    if(!readFile(fh.filename(),fh.flags)){
      ok = false;
    }
  }

  if(ok) num_readfiles_calls++;

  return(ok);
}

bool ConfigReader::isFileModified()
{
  unsigned i=0;
  while(!modified && i<files.size()){
    modified = files[i].watch.isFileModified();
    i++;
  }

  return(modified);
}

bool ConfigReader::needUpdate(int &client_generation) const
{
  if(client_generation != getGeneration()){
    client_generation = getGeneration();
    return(true);
  }else{
    return(false);
  }
}

void ConfigReader::eval(const char *exp)
{
  CharString fexp;
  fexp.printf("_ans=(%s);",exp);
  luaL_dostring(L,fexp());
  lua_getglobal(L,"_ans");
}

void ConfigReader::eval(const char *exp0,const char *exp1)
{
  CharString fexp;
  fexp.printf("_ans=(%s%s);",exp0,exp1);
  luaL_dostring(L,fexp());
  lua_getglobal(L,"_ans");
}

const char *ConfigReader::getStr(const char *exp,const char *default_val)
{
  eval(exp);

  const char *ret = NULL;
  if(lua_isstring(L,-1)){
    ret = lua_tostring(L,-1);
  }else{
    printf("ConfigReader: \"%s\" is not a string.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ret? ret : default_val);
}

bool ConfigReader::getBool(const char *exp,bool &val)
{
  eval(exp);

  bool ok = lua_isboolean(L,-1);
  if(ok){
    val = (bool)lua_toboolean(L,-1);
  }else{
    printf("ConfigReader: \"%s\" is not a boolean.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getInt(const char *exp,int &val)
{
  eval(exp);

  bool ok = lua_isnumber(L,-1);
  if(ok){
    val = (int)rint(lua_tonumber(L,-1));
  }else{
    printf("ConfigReader: \"%s\" is not an integer.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getUInt(const char *exp,unsigned int &val)
{
  eval(exp);

  bool ok = lua_isnumber(L,-1);
  if(ok){
    const int result = static_cast<int>(rint(lua_tonumber(L,-1)));
    val = static_cast<unsigned int>(result);
    if(result<0){
      printf("ConfigReader: \"%s\" is not an unsigned integer.\n",exp);
      errors++;
      ok = false;
    }
  }else{
    printf("ConfigReader: \"%s\" is not an integer.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getReal(const char *exp,float &val)
{
  eval(exp);

  bool ok = lua_isnumber(L,-1);
  if(ok){
    val = (float)lua_tonumber(L,-1);
  }else{
    printf("ConfigReader: \"%s\" is not a real number.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getReal(const char *exp,double &val)
{
  eval(exp);

  bool ok = lua_isnumber(L,-1);
  if(ok){
    val = (float)lua_tonumber(L,-1);
  }else{
    printf("ConfigReader: \"%s\" is not a real number.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getInt(const char *exp,int &val,int _min,int _max)
{
  if(!getInt(exp,val)) return(false);
  if(val<_min || val>_max){
    printf("ConfigReader: %s=%d is out of range [%d,%d].\n",
           exp,val,_min,_max);
    val = bound(val,_min,_max);
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::getPosReal(const char *exp,float &val)
{
  if(!getReal(exp,val)) return(false);
  if(val <= 0.0){
    printf("ConfigReader: %s=%f is non-positive\n",exp,val);
    val = 1E-6;
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::getPosReal(const char *exp,double &val)
{

  if(!getReal(exp,val)){
    printf("failure at %s\n",exp);
    return(false);
  }
  if(val <= 0.0){
    printf("ConfigReader: %s=%f is non-positive\n",exp,val);
    val = 1E-6;
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::getReal(const char *exp,float &val,float _min,float _max)
{
  if(!getReal(exp,val)) return(false);
  if(val<_min || val>_max){
    printf("ConfigReader: %s=%f is out of range [%f,%f].\n",
           exp,val,_min,_max);
    val = bound(val,_min,_max);
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::getReal(const char *exp,double &val,double _min,double _max)
{
  if(!getReal(exp,val)) return(false);
  if(val<_min || val>_max){
    printf("ConfigReader: %s=%f is out of range [%f,%f].\n",
           exp,val,_min,_max);
    val = bound(val,_min,_max);
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::set(const char *name,int val)
{
  CharString fexp;
  fexp.printf("%s=%d;",name,val);
  luaL_dostring(L,fexp());

  int v;
  return(getInt(name,v) && v==val);
}

bool ConfigReader::set(const char *name,double val)
{
  CharString fexp;
  fexp.printf("%s=%f;",name,val);
  luaL_dostring(L,fexp());

  double v;
  return(getReal(name,v) && fabs(val-v)<1E-9);
}

void ConfigReader::addStandard()
{
  // standard includes for a project go here
  addFile("config/common.cfg");
}

const char kLuaGetGlobalTables[] = "\
function _global_tables() \n \
  local str = \"\"; \n \
  for k,v in pairs(getfenv()) do \n \
    if type(v) == \"table\" then \n \
      str = str .. k .. \";\"; \n \
    end \n \
  end \n \
  return str; \n \
end \n \
";

bool ConfigReader::getGlobalsList(vector<string>* globals) {
  static const bool debug = false;
  CHECK_NOTNULL(globals);
  luaL_dostring(L,kLuaGetGlobalTables);
  const char exp[] = "_global_tables()";
  CharString fexp;
  fexp.printf("_ans=(%s);",exp);
  luaL_dostring(L,fexp());
  lua_getglobal(L,"_ans");

  const char *ret = NULL;
  if(lua_isstring(L,-1)){
    ret = lua_tostring(L,-1);
    if (debug) printf("Global vars: \n%s\n\n", ret);
    SplitKeys(ret, globals);
  }else{
    if (debug) printf("No global vars found.\n");
  }
  return true;
}

bool ConfigReader::getTableEntry (
    const char* exp, ConfigReader::LuaTableEntry* entry) {
  static const bool debug = false;
  CHECK_NOTNULL(entry);
  eval(exp);
  bool ok = true;
  if (lua_isboolean(L,-1)) {
    entry->type = LuaTableEntry::kTypeBoolean;
    entry->value = new bool(lua_toboolean(L,-1));
    ok = true;
    if (debug) printf("%s = %s\n", exp,
                      (*reinterpret_cast<bool*>(entry->value))?"true":"false");
  } else if (lua_isnumber(L,-1)) {
    entry->type = LuaTableEntry::kTypeNumber;
    entry->value = new double(lua_tonumber(L,-1));
    ok = true;
    if (debug) printf("%s = %f\n", exp,
                      *reinterpret_cast<double*>(entry->value));
  } else if (lua_isstring(L,-1)) {
    entry->type = LuaTableEntry::kTypeString;
    entry->value = new string(lua_tostring(L,-1));
    ok = true;
    if (debug) printf("%s = %s\n", exp,
                      reinterpret_cast<string*>(entry->value)->c_str());
  } else if (lua_istable(L,-1)) {
    entry->type = LuaTableEntry::kTypeTable;
    vector<LuaTableEntry>* ptr_table = new vector<LuaTableEntry>();
    entry->value = reinterpret_cast<void*>(ptr_table);
    if (debug) printf("%s = [Table]\n", exp);
    lua_pop(L,-1);
    ok = getTable(exp, ptr_table);
    return(ok);
  } else {
    printf("ConfigReader: \"%s\" is an unkown type.\n",exp);
    errors++;
    ok = false;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::isTable(const char* exp) {
  static const bool debug = false;
  if (debug) printf("Reading table %s\n", exp);
  eval(exp);
  const bool is_table = lua_istable(L,-1);
  lua_pop(L,-1);
  return(is_table);
}

bool ConfigReader::getTable (
    const char* exp, std::vector< ConfigReader::LuaTableEntry >* entries) {
  static const bool debug = false;
  CHECK_NOTNULL(entries);
  if (debug) printf("Reading table %s\n", exp);
  eval(exp);
  bool ok = lua_istable(L,-1);

  if(ok){
    if (debug) printf("%s is a valid table\n", exp);
    CharString fexp;
    //lua_pop(L,-1);
    fexp.printf("_ans=_table_entries(%s);",exp);
    luaL_dostring(L,fexp());
    lua_getglobal(L,"_ans");
    if(lua_isstring(L,-1)){
      const char* table_keys = lua_tostring(L,-1);
      lua_pop(L,-1);
      if (debug) printf("Table keys: %s\n", table_keys);
      vector<string> keys;
      SplitKeys(table_keys, &keys);
      for (unsigned int i = 0; i < keys.size(); ++i) {
        CharString table_entry_name;
        table_entry_name.printf("%s.%s", exp, keys[i].c_str());
        entries->push_back(LuaTableEntry());
        entries->back().key = keys[i];
        getTableEntry(table_entry_name(), &(entries->back()));
      }
    }else{
      printf("ConfigReader: \"%s\" is an invalid table.\n",exp);
      errors++;
    }
  }else{
    printf("ConfigReader: \"%s\" is not a table.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

