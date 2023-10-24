/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

//===========================================================================
//
// Parameter class - I use it frequently to read parameters from file or cmd line
//

namespace rai {
/** @brief A parameter that initializes itself from the command line
  (use \c rai::init), parameter file, or a default value (priority in
  this order).  Initialization is done on the fly the _first_ time
  its value is queried (i.e., referenced by the cast operators).*/
template<class type>
class Parameter {
 public:
  const char* typeName;
  type value, Default;
  const char* tag;
  bool initialized, hasDefault;

 public:
  /// @name constructors

  /// Determines the tag to search for in parameter file/command line
  explicit Parameter(const char* _tag) {
    typeName=typeid(type).name();
    initialized=false;
    tag=_tag;
    hasDefault=false;
  };

  /** @brief specifies also a default value -- parameter does not have to but
    can be specified in the parameter file/command line */
  Parameter(const char* _tag, const type& _default) {
    typeName=typeid(type).name();
    initialized=false;
    tag=_tag;
    hasDefault=true;
    Default=_default;
  };

  ~Parameter() {}

  /// @name value access

  /// standard type conversion: returns a const of the parameter value
  operator type() { if(!initialized) initialize(); return value; }

  /// ()-operator: returns an lvalue of the parameter value
  type& operator()() { if(!initialized) initialize(); return value; }

  /// @name manipulation

  /// assigs a value to the parameter -- no further initialization needed
  type& operator=(const type v) { initialized=true; value=v; return value; }

  /// set the tag (replacing the one from the constructor)
  void setTag(char* _tag) { tag=_tag; }

  /** @brief enforces that the parameter is reinitialized from the parameter
    file/command line, the next time it is referenced -- even if it
    has been initialized before */
  void reInitialize() { initialized=false; }

  /// @name explicit grabbing

 private:
  void initialize();
};

}

template<class T> void Parameter<T>::initialize() {
  if(!initialized) {
    getParameterBase(value, tag, hasDefault, &Default);
    initialized = true;
  }
}

/** @brief a standard method to save an object into a file. The same as
  std::ofstream file; rai::open(file, filename); file <<x;
  file.close(); */
template<class T> void save(const T& x, const char* filename) {
  std::ofstream file;
  open(file, filename);
  file <<x;
  file.close();
}

/** @brief a standard method to load object from a file. The same as
std::ifstream file; rai::open(file, filename); file >>x;
file.close(); */
template<class T> void load(T& x, const char* filename, bool change_directory) {
#ifdef RAI_MSVC
  if(change_directory) RAI_MSG("can't handle change_directory with MSVC");
  change_directory = false;
#endif
  if(!change_directory) {
    std::ifstream file;
    open(file, filename);
    file >>x;
    file.close();
  } else {
    FILE(filename) >>x;
  }
}
