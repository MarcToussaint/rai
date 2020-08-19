/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

template void rai::Parameter<rai::String>::initialize();
template void rai::Parameter<bool>::initialize();
template void rai::Parameter<double>::initialize();
template void rai::Parameter<int>::initialize();
template void rai::Parameter<uint>::initialize();

/** @brief Open a (possibly new) config file with name '\c name'.<br> If
  \c name is not specified, it searches for a command line-option
  '-cfg' and, if not found, it assumes \c name=rai.cfg */
void openConfigFile(const char* name) {
  LOG(3) <<"opening config file ";
  if(!name) name=getCmdLineArgument("cfg");
  if(!name) name=RAI_ConfigFileName;
  if(globalThings().cfgFileOpen) {
    globalThings().cfgFile.close(); LOG(3) <<"(old config file closed) ";
  }
  LOG(3) <<"'" <<name <<"'";
  globalThings().cfgFile.clear();
  globalThings().cfgFile.open(name);
  globalThings().cfgFileOpen=true;
  if(!globalThings().cfgFile.good()) {
    //RAI_MSG("couldn't open config file " <<name);
    LOG(3) <<" - failed";
  }
  LOG(3) <<std::endl;
}

/** @brief Search the first occurence of a sequence '\c tag:'
in the config file (opened automatically) and, if found, pipes
it in \c value. Returns false if parameter is not found. */
template<class T>
bool getFromCfgFile(T& x, const char* tag) {
  globalThings().cfgFileMutex.lock();
  if(!globalThings().cfgFileOpen) openConfigFile();
  globalThings().cfgFile.clear();
  globalThings().cfgFile.seekg(std::ios::beg);
  if(!globalThings().cfgFile.good()) { globalThings().cfgFileMutex.unlock(); return false; }
  unsigned n=strlen(tag);
  char* buf=new char [n+2]; memset(buf, 0, n+2);
  while(globalThings().cfgFile.good()) {
    memmove(buf, buf+1, n);
    buf[n]=globalThings().cfgFile.get();
    if(buf[n]==' ' || buf[n]=='\t' || buf[n]==':' || buf[n]=='=') { buf[n]=0; if(!strcmp(tag, buf)) break; buf[n]=':'; }
  };
  delete[] buf;

  if(!globalThings().cfgFile.good()) { globalThings().cfgFileMutex.unlock(); return false; }

  skip(globalThings().cfgFile, " :=\n\r\t");
  globalThings().cfgFile >>x;

  if(globalThings().cfgFile.fail()) HALT("error when reading parameter " <<tag);
  globalThings().cfgFileMutex.unlock();
  return true;
}

template<class T> //von Tim Rackowski
struct ParameterMap {
  static std::map<std::string, T> m;
};

template<class T> std::map<std::string, T> ParameterMap<T>::m;

template<class T>
void putParameter(const char* tag, const T& x) {
  ParameterMap<T>::map[tag] = x;
}

template <class T>
bool getFromMap(T& x, const char* tag) {
  typename std::map<std::string, T>::const_iterator p = ParameterMap<T>::m.find(tag);
  if(p == ParameterMap<T>::m.end())
    return false;
  x = p->second;
  return true;
}

template std::map<std::string, int> rai::ParameterMap<int>::m;
template std::map<std::string, double> rai::ParameterMap<double>::m;
template std::map<std::string, unsigned int> rai::ParameterMap<unsigned int>::m;
template std::map<std::string, float> rai::ParameterMap<float>::m;
template std::map<std::string, bool> rai::ParameterMap<bool>::m;
template std::map<std::string, long> rai::ParameterMap<long>::m;
template std::map<std::string, rai::String> rai::ParameterMap<rai::String>::m;
template std::map<std::string, std::string> rai::ParameterMap<std::string>::m;

