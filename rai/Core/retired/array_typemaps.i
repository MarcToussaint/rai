%inline %{
  #include <Core/array.h>
  #include <Core/array_t.h>
  #define SWIG_FILE_WITH_INIT
%}

%include "numpy.i"
%fragment("NumPy_Fragments");
%init %{
import_array();
%}

//===========================================================================
// MT::Array wrapper to numpy
//===========================================================================
// For numerical types, we want to get numpy arrays, to keep the shape of the
// array, etc.


// actual translation of numpy array to MT::Array<double>
%fragment("ArrayTransform", "header") {
  template<class T>
  void asMTArray(MT::Array<T>& result, PyObject *nparray, int type) {
    uint size = 1;
    for(uint i=0; i<array_numdims(nparray); ++i)
      size *= array_size(nparray, i);
    result.resize(size);

    PyArrayObject* src = (PyArrayObject*) nparray;

    // cast array entries to the correct type if necessary
    if (PyArray_TYPE(src) != type) {
      if (PyArray_CanCastSafely(PyArray_TYPE(src), type)) {
        src = (PyArrayObject*) PyArray_SimpleNew(array_numdims(nparray), array_dimensions(nparray), type);
        PyArray_CastTo(src, (PyArrayObject*) nparray);
      }
      else {
        PyErr_SetString(PyExc_TypeError, "Not an array of compatible numeric values");
      }
    }

    // Copy actual data
    memcpy(result.p, PyArray_DATA(src), size*sizeof(T));

    // reshape to the correct size
    if(array_numdims(nparray) == 1)
      result.reshape(array_size(nparray, 0));
    else if(array_numdims(nparray) == 2)
      result.reshape(array_size(nparray, 0), array_size(nparray, 1));
    else if(array_numdims(nparray) == 3)
      result.reshape(array_size(nparray, 0), array_size(nparray, 1), array_size(nparray, 2));
    else
      PyErr_SetString(PyExc_TypeError, "MT::Array does not support matrices with more than 3 dimensions.");
  }
}


//===========================================================================
// The typemap macro, to get all our types done with one piece of code
//===========================================================================
%define %Array_Typemap(Type)

// Calls the transform template with the right numpy type etc.
%fragment("asMTArray"{Type}, "header", fragment="ArrayTransform", fragment="getNP_TYPE"{Type}) {
  void asMTArray(MT::Array<Type>& result, PyObject *nparray) {
    asMTArray(result, nparray, numpy_type_##Type());
  }
}

//===========================================================================
// The actual typemaps for value, reference and pointer arguments
//===========================================================================
%typemap(in, fragment="asMTArray"{Type}) MT::Array<Type> {
  if(is_array($input)) {
    asMTArray($1, $input);
  }
  else {
    PyErr_SetString(PyExc_TypeError, "Not a numpy array");
    return NULL;
  }
}

%typemap(in, fragment="asMTArray"{Type}) MT::Array<Type> & {
  if(is_array($input)) {
    $1 = new MT::Array<Type>();
    asMTArray(*$1, $input);
  }
  else {
    PyErr_SetString(PyExc_TypeError, "Not a numpy array");
    return NULL;
  }
}



//===========================================================================
// Garbage collection
//===========================================================================

%typemap(freearg) MT::Array<Type> & {
  delete $1;
}

//===========================================================================
// members
//===========================================================================

%typemap(memberin) MT::Array<Type> {
  $1 = $input;  
}

%typemap(memberin) MT::Array<Type> & {
  $1 = new MT::Array<Type>;
  *$1 = *$input;  
}
//===========================================================================
// Typecheck typemaps for all kinds of overloading magic
//===========================================================================
%typemap(typecheck) MT::Array<Type> {
  if(is_array($input)) $1 = 1;
  else $1 = 0;
}

%typemap(typecheck) MT::Array<Type> & = MT::Array<Type>;

//===========================================================================
// Out typemaps 
//===========================================================================
%fragment("out_tm"{Type}, "header", fragment="getNP_TYPE"{Type}) {
  PyObject *MTArray_As_NumpyArray(MT::Array<Type> in) {
    long dims[3] = { in.d0, in.d1, in.d2 };
    PyArrayObject *a = (PyArrayObject*) PyArray_SimpleNew(in.nd, dims, numpy_type_##Type());
    memcpy(PyArray_DATA(a), in.p, in.N*sizeof(Type));
    /*a->data = reinterpret_cast<char*>(in.p);*/
    return PyArray_Return(a);
  }
}

%typemap(out, fragment="out_tm"{Type}) MT::Array<Type> {
  $result = MTArray_As_NumpyArray($1);
}

%typemap(out, fragment="out_tm"{Type}) MT::Array<Type> & {
  $result = MTArray_As_NumpyArray(*$1);
}

%typemap(argout, fragment="out_tm"{Type}) MT::Array<Type> & {
  %append_output(MTArray_As_NumpyArray(*$1));
}

// const types can't be outputs (or they are pretty useless anyway)
%typemap(argout, fragment="out_tm"{Type}) const MT::Array<Type> & {
}

%apply MT::Array<Type> & { MT::Array<Type> * }

%enddef

//===========================================================================
// List Wrapper
//===========================================================================
// MT::Array is also used as list datatype. This is kind of a functional
// overload. However, we translate the list-like types to normal python Lists,
// whereas we translate arr, intA and uintA to their corresponding numpy array.
// We could translate everything to Lists, but the we would loose the shape
// information.
// BEWARE: if you define a array of typemapped types you won't get what you
// expect. (But you will get a list of not typemapped SWIG_Objects.)


%fragment("asMTArrayList", "header") {
  template<class T>
  int asMTArrayList(MT::Array<T>& list, PyObject* pylist, swig_type_info* swigtype) {
    list.resize(PyList_Size(pylist));
    for(uint i=0; i<PyList_Size(pylist); ++i) {
      T obj;
      int res = SWIG_ConvertPtr(PyList_GetItem(pylist, i), (void**) &obj, swigtype, 0);
      if(SWIG_IsOK(res)) {
        list(i) = obj;
      }
      else {
        PyErr_SetString(PyExc_TypeError, "Not a list of compatible objects.");
        return 0;
      }
    } 
    return 1;
  }
}

//===========================================================================
// The macro to generate typemaps for new lists
//===========================================================================
%define %List_Typemap(Type)

//===========================================================================
// The actual typemaps for value, reference and pointer arguments
//===========================================================================
// This is super hacky. Kids don't try at home. I abused the $typemap macro to
// get some kind of recursive typemaps. It would be much nicer, to put it in a
// fragment or so, but SWIG doesn't provide that functionality.

%typemap(in, fragment="asMTArrayList") MT::Array<Type*> {
  if(PyList_Check($input)) {
    $1.resize(PyList_Size($input));
    for(uint i=0; i<PyList_Size($input); ++i) {
      Type *tm;
      PyObject *iter = PyList_GetItem($input, i);
      {
        Type *$1;
        PyObject *$input = iter;
        $typemap(in, Type*)
        tm = $1;
      }
      $1(i) = tm;
    }
  }
  else {
    // the typecheck typemap should ensure, that this never happens
    PyErr_SetString(PyExc_TypeError, "Not a list");
    return NULL;
  }
}

%typemap(in, fragment="asMTArrayList") MT::Array<Type*> & {
  if(PyList_Check($input)) {
    $1 = new MT::Array<Type*>(PyList_Size($input));
    for(uint i=0; i<PyList_Size($input); ++i) {
      Type *tm;
      PyObject *iter = PyList_GetItem($input, i);
      {
        Type *$1;
        PyObject *$input = iter;
        $typemap(in, Type*)
        tm = $1;
      }
      (*$1)(i) = tm;
    }
  }
  else {
    // the typecheck typemap should ensure, that this never happens
    PyErr_SetString(PyExc_TypeError, "Not a list");
    return NULL;
  }
}


//===========================================================================
// Output
//===========================================================================

%typemap(out) MT::Array<Type*> {
  $result = PyList_New($1.N);
  for(uint i=0; i<$1.N; ++i) {
    PyObject *obj = NULL;
    Type **iter = &($1(i));
    {
      Type *$1 = *iter;
      PyObject *$result = NULL;

      // this sets result.
      $typemap(out, Type*)

      obj = $result; 
    }
    PyList_SetItem($result, i, obj);
  }
}

%typemap(out) MT::Array<Type*> & {
  $result = PyList_New($1->N);
  for(uint i=0; i<$1->N; ++i) {
    PyObject *obj = NULL;
    Type **iter = &((*$1)(i));
    {
      Type *$1 = *iter;
      PyObject *$result = NULL;
      
      // this sets result.
      $typemap(out, Type*)

      obj = $result; 
    }
    PyList_SetItem($result, i, obj);
  }
}

%typemap(argout) MT::Array<Type*> & {
  PyObject *argout = PyList_New($1->N);
  for(uint i=0; i<$1->N; ++i) {
    PyObject *obj = NULL;
    Type **iter = &((*$1)(i));
    {
      Type *$1 = *iter;
      int $owner = 0; // this is hacky
      PyObject *$result = NULL;

      // this sets result.
      $typemap(out, Type*)

      obj = $result; 
    }
    PyList_SetItem(argout, i, obj);
  }
  %append_output(argout);
}

%typemap(argout) const MT::Array<Type*> & { }

//===========================================================================
// Garbage collection
//===========================================================================

%typemap(freearg) MT::Array<Type*> & {
  delete $1;
}

//===========================================================================
// members
//===========================================================================

%typemap(memberin) MT::Array<Type*> {
#ifdef SWIG_DEREFERENCE_MEMBERS
  $1 = *$input;
#else
  $1 = $input;
#endif
}

%typemap(memberin) MT::Array<Type*> & {
  $1 = new MT::Array<Type*>;
  *$1 = *$input;  
}

//===========================================================================
// Typechecking for overload magic 
//===========================================================================

%typemap(typecheck) MT::Array<Type*> {
  if (PyList_Check($input)) {
    Type *obj;
    int res = SWIG_ConvertPtr(PyList_GetItem($input, 0), (void**) &obj, $descriptor(Type*), 0);
    if (SWIG_IsOK(res)) $1 = 1;
    else $1 = 0;
  }
  else $1 = 0;
}

/*%typemap(typecheck) MT::Array<Type*> & = MT::Array<Type*>;*/

/*%apply MT::Array<Type*> & { MT::Array<Type*> * }*/

%enddef
