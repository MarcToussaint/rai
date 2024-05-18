/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "h5.h"

#ifdef RAI_H5

#include <H5Cpp.h>

//===========================================================================

template<class T> H5::DataType get_h5type();
template<> H5::DataType get_h5type<double>() { return H5::PredType::NATIVE_DOUBLE; }
template<> H5::DataType get_h5type<float>() { return H5::PredType::NATIVE_FLOAT; }
template<> H5::DataType get_h5type<int>() { return H5::PredType::NATIVE_INT; }
template<> H5::DataType get_h5type<uint>() { return H5::PredType::NATIVE_UINT; }
template<> H5::DataType get_h5type<int16_t>() { return H5::PredType::NATIVE_INT16; }
template<> H5::DataType get_h5type<uint16_t>() { return H5::PredType::NATIVE_UINT16; }
template<> H5::DataType get_h5type<char>() { return H5::PredType::NATIVE_CHAR; }
template<> H5::DataType get_h5type<unsigned char>() { return H5::PredType::NATIVE_UCHAR; }

//===========================================================================

H5_Writer::H5_Writer(const char* filename) {
  file = make_shared<H5::H5File>(filename, H5F_ACC_TRUNC);
}

void H5_Writer::addGroup(const char* group) {
  file->createGroup(group);
}

template<class T> void H5_Writer::add(const char* name, const rai::Array<T>& x) {
  rai::Array<hsize_t> dim = rai::convert<hsize_t>(x.dim());
  H5::DataSpace dataspace(dim.N, dim.p);
  H5::DataType h5type = get_h5type<T>();
  H5::DataSet dataset = file->createDataSet(name, h5type, dataspace);
  dataset.write(x.p, h5type);
}

//===========================================================================

H5_Reader::H5_Reader(const char* filename) {
  file = make_shared<H5::H5File>(filename, H5F_ACC_RDONLY);
}

uintA get_dim(H5::DataSet& dataset) {
  H5::DataSpace dataspace = dataset.getSpace();
  rai::Array<hsize_t> _dim(dataspace.getSimpleExtentNdims());
  dataspace.getSimpleExtentDims(_dim.p, NULL);
  return rai::convert<uint>(_dim);
}

bool H5_Reader::exists(const char* name) {
  return file->nameExists(name);
}

template<class T> rai::Array<T> H5_Reader::read(const char* name, bool ifExists) {
  if(ifExists && !exists(name)) return {};
  H5::DataSet dataset = file->openDataSet(name);
  rai::Array<T> x;
  x.resize(get_dim(dataset));
  dataset.read(x.p, get_h5type<T>());
  return x;
}

template<class T> void readDatasetToGraph(rai::Graph& G, H5::DataSet& dataset, const uintA& dim, const char* name) {
  rai::Node_typed<rai::Array<T>>* x = G.add<rai::Array<T>>(name);
  x->value.resize(dim);
  dataset.read(x->value.p, get_h5type<T>());
}

herr_t file_callback(hid_t loc_id, const char* name, const H5L_info_t* linfo, void* Lptr) {
  H5_Reader* L = (H5_Reader*)Lptr;
  if(L->verbose) cout <<"== loading: " <<name <<endl;

  H5::DataSet dataset = L->file->openDataSet(name);
  uintA dim = get_dim(dataset);
  if(L->verbose) cout <<"   dim: " <<dim << endl;

  H5T_class_t type_class = dataset.getTypeClass();

  if(type_class == H5T_FLOAT) {
    H5::FloatType type = dataset.getFloatType();
    if(L->verbose) cout <<"   float type: " <<type.getOrder() <<' ' <<type.getPrecision() <<' ' <<type.getSize() <<endl;
    uint bits = type.getPrecision();
    if(bits==64) readDatasetToGraph<double>(L->G, dataset, dim, name);
    else if(bits==32) readDatasetToGraph<float>(L->G, dataset, dim, name);
    else NIY;

  } else if(type_class == H5T_INTEGER) {
    H5::IntType type = dataset.getIntType();
    if(L->verbose) cout <<"   integer type: " <<type.getOrder() <<' ' <<type.getPrecision() <<' ' <<type.getSize() <<' ' <<type.getSign() <<endl;
    uint bits = type.getPrecision();
    H5T_sign_t sign = type.getSign();
    if(bits==32 && sign) readDatasetToGraph<int>(L->G, dataset, dim, name);
    else if(bits==32 && !sign) readDatasetToGraph<uint>(L->G, dataset, dim, name);
    else if(bits==16 && sign) readDatasetToGraph<int16_t>(L->G, dataset, dim, name);
    else if(bits==16 && !sign) readDatasetToGraph<uint16_t>(L->G, dataset, dim, name);
    else NIY;
  } else {
    NIY;
  }

  return 0;
}

void H5_Reader::readAll() {
  H5Literate(file->getId(), H5_INDEX_NAME, H5_ITER_INC, NULL, file_callback, this);
}

#else

H5_Writer::H5_Writer(const char* filename) { NICO }
void H5_Writer::addGroup(const char* group) { NICO }
template<class T> void H5_Writer::add(const char* name, const rai::Array<T>& x) { NICO }
H5_Reader::H5_Reader(const char* filename) { NICO }
template<class T> rai::Array<T> H5_Reader::read(const char* name, bool ifExists) { NICO }
bool H5_Reader::exists(const char* name) { NICO }

#endif

// explicit instantiations
template void H5_Writer::add<double>(const char* name, const rai::Array<double>& x);
template void H5_Writer::add<float>(const char* name, const rai::Array<float>& x);
template void H5_Writer::add<int>(const char* name, const rai::Array<int>& x);
template void H5_Writer::add<uint>(const char* name, const rai::Array<uint>& x);
template void H5_Writer::add<int16_t>(const char* name, const rai::Array<int16_t>& x);
template void H5_Writer::add<uint16_t>(const char* name, const rai::Array<uint16_t>& x);
template void H5_Writer::add<char>(const char* name, const rai::Array<char>& x);
template void H5_Writer::add<unsigned char>(const char* name, const rai::Array<unsigned char>& x);

template rai::Array<double> H5_Reader::read<double>(const char* name, bool ifExists);
template rai::Array<float> H5_Reader::read<float>(const char* name, bool ifExists);
template rai::Array<int> H5_Reader::read<int>(const char* name, bool ifExists);
template rai::Array<uint> H5_Reader::read<uint>(const char* name, bool ifExists);
template rai::Array<int16_t> H5_Reader::read<int16_t>(const char* name, bool ifExists);
template rai::Array<uint16_t> H5_Reader::read<uint16_t>(const char* name, bool ifExists);
template rai::Array<char> H5_Reader::read<char>(const char* name, bool ifExists);
template rai::Array<byte> H5_Reader::read<byte>(const char* name, bool ifExists);
