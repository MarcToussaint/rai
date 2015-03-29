/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef MT_keyValueGraph_h
#define MT_keyValueGraph_h

#include "array.h"
#include <map>

struct Item;
struct Graph;
typedef MT::Array<Item*> ItemL;
extern ItemL& NoItemL; //this is a pointer to NULL! I use it for optional arguments
extern Graph& NoGraph; //this is a pointer to NULL! I use it for optional arguments

//===========================================================================

struct Item {
  Graph& container;
  StringA keys;
  ItemL parents;
  ItemL parentOf;
  uint index;
  Item(Graph& _container);
  Item(Graph& _container, const ItemL& _parents);
  virtual ~Item();
  template<class T> T *getValue();    ///< query whether the Item is of a certain value, return the value if so
  template<class T> const T *getValue() const; ///< as above
  template<class T> T& V(){ T *x=getValue<T>(); CHECK(x,"wrong type"); return *x; }

  bool matches(const char *key);
  bool matches(const StringA &query_keys);
  void write(std::ostream &os) const;
  Graph ParentOf();
  //-- specific standard values TODO: make return pointer!
  Graph& kvg(){ Graph *kvg=getValue<Graph>(); CHECK(kvg,""); return *kvg; }

  //-- virtuals implemented by Item_typed
  virtual bool hasValue() const {NIY}
  virtual void *getValueDirectly() const {NIY}
  virtual void writeValue(std::ostream &os) const {NIY}
  virtual const std::type_info& getValueType() const {NIY}
  virtual bool is_derived_from_RootType() const {NIY}
  virtual void copyValue(Item*) {NIY}
  virtual void takeoverValue(Item*) {NIY}
  virtual bool hasEqualValue(Item*) {NIY}
  virtual Item *newClone(Graph& container) const {NIY}
};
stdOutPipe(Item);

//===========================================================================

struct ItemInitializer{
  ItemInitializer(const char* key);
  template<class T> ItemInitializer(const char* key, const T& x);
  template<class T> ItemInitializer(const char* key, const StringA& parents, const T& x);
  Item *it;
  StringA parents;
};

//===========================================================================

struct Graph:ItemL {
  struct sKeyValueGraph *s;
  Graph* isReferringToItemsOf; //TODO: remove
  Item *isItemOfParentKvg;
  
  Graph();
  explicit Graph(const char* filename);
  Graph(const std::map<std::string, std::string>& dict);
  Graph(std::initializer_list<ItemInitializer> list);
  Graph(const Graph& G);
//  Graph(Item *itemOfParentKvg);
  ~Graph();
  
  Graph& operator=(const Graph&);
  void clear();
  ItemL& list() { return *this; }
  
  //-- get items
  Item* getItem(const char *key) const;
  Item* getItem(const char *key1, const char *key2);
  Item* getItem(const StringA &keys);
  Item* operator[](const char *key) { return getItem(key); }
  Item& I(const char *key) { Item *it=getItem(key); CHECK(it,"item '" <<key <<"' does not exist"); return *it; }
  Item* getChild(Item *p1, Item *p2) const;

  //-- get lists of items (TODO: return ItemL, not referring Graph)
  Graph getItems(const char* key);
  Graph getItemsOfDegree(uint deg);
  Graph getTypedItems(const char* key, const std::type_info& type);
  template<class T> Graph getTypedItems(const char* key){ return getTypedItems(key, typeid(T)); }
  template<class T> ItemL getDerivedItems();

  //-- get values directly (TODO: remove)
  template<class T> T* getValue(const char *key);
  template<class T> T* getValue(const StringA &keys);
  template<class T> bool getValue(T& x, const char *key) { T* y=getValue<T>(key); if(y) { x=*y; return true; } return false; }
  template<class T> bool getValue(T& x, const StringA &keys) { T* y=getValue<T>(keys); if(y) { x=*y; return true; } return false; }

  //-- get lists of all values of a certain type T (or derived from T)
  template<class T> MT::Array<T*> getTypedValues(const char* key);
  template<class T> MT::Array<T*> getDerivedValues();
  
  //-- adding items
  template<class T> Item *append(T *x, bool ownsValue);
  template<class T> Item *append(const char* key, T *x, bool ownsValue);
  template<class T> Item *append(const StringA& keys, const ItemL& parents, T *x, bool ownsValue);
//  template<class T> Item *append(const StringA& keys, T *x, bool ownsValue) { return append(keys, ItemL(), x, ownsValue); }
//  template<class T> Item *append(const char *key, T *x, bool ownsValue) { return append({MT::String(key)}, ItemL(), x, ownsValue); }
//  template<class T> Item *append(const char *key1, const char* key2, T *x, bool ownsValue) {  return append({MT::String(key1), MT::String(key2)}, ItemL(), x, ownsValue); }
  Item *append(const uintA& parentIdxs);

  void appendDict(const std::map<std::string, std::string>& dict);

  //-- merging items  //TODO: explain better
  Item *merge(Item* m); //removes m and deletes, if it is a member of This and merged with another Item
  void merge(const ItemL& L){ for(Item *m:L) merge(m); }

  //-- debugging
  bool checkConsistency() const;

  //-- indexing
  uint index(bool subKVG=false, uint start=0); //TODO: make private

  //-- I/O
  void sortByDotOrder();
  
  void read(std::istream& is);
  void write(std::ostream& os=std::cout, const char *ELEMSEP="\n", const char *delim=NULL) const;
  void writeDot(std::ostream& os, bool withoutHeader=false, bool defaultEdges=false, int nodesOrEdges=0);
};
stdPipes(Graph);

//===========================================================================

inline Graph GRAPH(const ItemL& L){ //TODO: remove
  Graph G;
  G.isReferringToItemsOf = (Graph*)(1);
  G.ItemL::operator=(L);
  return G;
}

inline bool ItemComp(Item* const& a, Item* const& b){ //TODO: why?
  return a < b;
}

#include "graph_t.h"

//===========================================================================
//
// Andrea's util based on Graph
// (would put in util.h, but creates inclusion loop which breaks compilation)
//

// Params {{{
#include "graph.h"
struct Params {
  Graph kvg;

  template<class T>
  void set(const char *key, const T &value) {
    Item *i = kvg.getItem(key);
    if(i) *i->getValue<T>() = value;
    else kvg.append({key}, {}, new T(value), true);
  }

  template<class T>
  bool get(const char *key, T &value) { return kvg.getValue(value, key); }

  template<class T>
  T* get(const char *key) { return kvg.getValue<T>(key); }

  void clear() { kvg.clear(); }

  bool remove(const char *key) {
    delete kvg[key];
    Item *i = kvg.getItem(key);
    if(!i) return false;
    delete i;
//    // TODO is list() here necessary?
//    kvg.list().remove(i->index);
    return true;
  }

  void write(std::ostream &os = std::cout) const {
    os << "params = {" << std::endl;
    kvg.write(os, " ");
    os << "}" << std::endl;
  }
};
stdOutPipe(Params)
// }}}
// Parametric {{{
struct Parametric {
  Params params;
};
// }}}

#endif

/// @} //end group
