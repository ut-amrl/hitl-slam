#include "configreader.h"

int main(int argc,char **argv)
{
  if(argc != 3) return(-1);

  WatchFiles watch_files;
  ConfigReader config;
  const char *filename = argv[1];
  const char *key      = argv[2];

  config.init(watch_files);
  config.addFile("config/nonexist.cfg",ConfigReader::Optional);
  config.addFile(filename);
  if(!config.readFiles()){
    printf("Failed to read config\n");
    exit(1);
  }

  bool loop = true;

  do{
    // print the variable value
    const char *value = config.getStr(key,"");
    printf("%s = \"%s\"\n",key,value);

    if(loop){
      // poll for a change to happen, then reread config
      while(watch_files.getEvents() == 0) usleep(100*1000);
      if(config.isFileModified()) config.readFiles();
      watch_files.clearEvents();
    }
  }while(loop);

  return(0);
}
