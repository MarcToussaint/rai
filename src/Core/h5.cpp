#include "h5.h"

#include <H5Cpp.h>

//===========================================================================

template<class T> H5::DataType get_h5type();
template<> H5::DataType get_h5type<double>(){ return H5::PredType::NATIVE_DOUBLE; }
template<> H5::DataType get_h5type<float>(){ return H5::PredType::NATIVE_FLOAT; }
template<> H5::DataType get_h5type<int>(){ return H5::PredType::NATIVE_INT; }
template<> H5::DataType get_h5type<uint>(){ return H5::PredType::NATIVE_UINT; }
template<> H5::DataType get_h5type<int16_t>(){ return H5::PredType::NATIVE_INT16; }
template<> H5::DataType get_h5type<uint16_t>(){ return H5::PredType::NATIVE_UINT16; }

//===========================================================================

H5_Writer::H5_Writer(const char* filename){
  file = make_shared<H5::H5File>(filename, H5F_ACC_TRUNC);
}

template<class T> void H5_Writer::add(const char* name, const rai::Array<T>& x){
  rai::Array<hsize_t> dim = rai::convert<hsize_t>(x.dim());
  H5::DataSpace dataspace(dim.N, dim.p);
  H5::DataType h5type = get_h5type<T>();
  H5::DataSet dataset = file->createDataSet(name, h5type, dataspace);
  dataset.write(x.p, h5type);
}

// explicit instantiations
template void H5_Writer::add<double>(const char* name, const rai::Array<double>& x);
template void H5_Writer::add<float>(const char* name, const rai::Array<float>& x);
template void H5_Writer::add<int>(const char* name, const rai::Array<int>& x);
template void H5_Writer::add<uint>(const char* name, const rai::Array<uint>& x);
template void H5_Writer::add<int16_t>(const char* name, const rai::Array<int16_t>& x);
template void H5_Writer::add<uint16_t>(const char* name, const rai::Array<uint16_t>& x);

//===========================================================================

H5_Reader::H5_Reader(const char* filename, bool readAll) {
  file = make_shared<H5::H5File>(filename, H5F_ACC_RDONLY);
  if(readAll) this->readAll();
}

template<class T> void readDatasetToGraph(rai::Graph& G, H5::DataSet& dataset, const uintA& dim, const char* name){
  rai::Node_typed<rai::Array<T>>* x = G.add<rai::Array<T>>(name);
  x->value.resize(dim);
  dataset.read(x->value.p, get_h5type<T>());
}

herr_t file_callback(hid_t loc_id, const char* name, const H5L_info_t* linfo, void* Lptr){
  H5_Reader *L = (H5_Reader*)Lptr;
  if(L->verbose) cout <<"== loading: " <<name <<endl;

  H5::DataSet dataset = L->file->openDataSet(name);
  H5::DataSpace dataspace = dataset.getSpace();
  rai::Array<hsize_t> _dim(dataspace.getSimpleExtentNdims());
  dataspace.getSimpleExtentDims(_dim.p, NULL);
  uintA dim = rai::convert<uint>(_dim);
  if(L->verbose) cout <<"   dim: " <<dim << endl;

  H5T_class_t type_class = dataset.getTypeClass();

  if (type_class == H5T_FLOAT) {
    H5::FloatType type = dataset.getFloatType();
    if(L->verbose) cout <<"   float type: " <<type.getOrder() <<' ' <<type.getPrecision() <<' ' <<type.getSize() <<endl;
    uint bits = type.getPrecision();
    if(bits==64) readDatasetToGraph<double>(L->G, dataset, dim, name);
    else if(bits==32) readDatasetToGraph<float>(L->G, dataset, dim, name);
    else NIY;

  } else if (type_class == H5T_INTEGER) {
    H5::IntType type = dataset.getIntType();
    if(L->verbose) cout <<"   integer type: " <<type.getOrder() <<' ' <<type.getPrecision() <<' ' <<type.getSize() <<' ' <<type.getSign() <<endl;
    uint bits = type.getPrecision();
    H5T_sign_t sign = type.getSign();
    if(bits==32 && sign) readDatasetToGraph<int>(L->G, dataset, dim, name);
    else if(bits==32 && !sign) readDatasetToGraph<uint>(L->G, dataset, dim, name);
    else if(bits==16 && sign) readDatasetToGraph<int16_t>(L->G, dataset, dim, name);
    else if(bits==16 && !sign) readDatasetToGraph<uint16_t>(L->G, dataset, dim, name);
    else NIY;
  }else{
    NIY;
  }

  return 0;
}

void H5_Reader::readAll(){
  H5Literate(file->getId(), H5_INDEX_NAME, H5_ITER_INC, NULL, file_callback, this);
}
