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

struct Node;
struct Graph;
struct ParseInfo;
struct GraphEditCallback;
typedef MT::Array<Node*> NodeL;
typedef MT::Array<ParseInfo*> ParseInfoL;
typedef MT::Array<GraphEditCallback*> GraphEditCallbackL;
extern NodeL& NoItemL; //this is a pointer to NULL! I use it for optional arguments
extern Graph& NoGraph; //this is a pointer to NULL! I use it for optional arguments

//===========================================================================

struct Node {
  Graph& container;
  StringA keys;
  NodeL parents;
  NodeL parentOf;
  uint index;
  Node(Graph& _container);
  Node(Graph& _container, const StringA& _keys, const NodeL& _parents);
  virtual ~Node();
  template<class T> T *getValue();    ///< query whether the Node is of a certain value, return the value if so
  template<class T> const T *getValue() const; ///< as above
  template<class T> T& V(){ T *x=getValue<T>(); CHECK(x,"wrong type"); return *x; }

  bool matches(const char *key);
  bool matches(const StringA &query_keys);
  void write(std::ostream &os) const;
  Graph ParentOf();
  //-- specific standard values
  Graph& graph(){ Graph *kvg=getValue<Graph>(); CHECK(kvg,""); return *kvg; }

  //-- virtuals implemented by Node_typed
  virtual bool hasValue() const {NIY}
  virtual void* getValueDirectly() const {NIY}
  virtual void writeValue(std::ostream &os) const {NIY}
  virtual const std::type_info& getValueType() const {NIY}
  virtual bool is_derived_from_RootType() const {NIY}
  virtual void copyValue(Node*) {NIY}
  virtual void takeoverValue(Node*) {NIY}
  virtual bool hasEqualValue(Node*) {NIY}
  virtual Node* newClone(Graph& container) const {NIY}
};
stdOutPipe(Node);

//===========================================================================

struct ItemInitializer{
  ItemInitializer(const char* key);
  template<class T> ItemInitializer(const char* key, const T& x);
  template<class T> ItemInitializer(const char* key, const StringA& parents, const T& x);
  Node *it;
  StringA parents;
};

//===========================================================================

struct Graph:NodeL {
  struct sKeyValueGraph *s;
  Graph* isReferringToItemsOf; //TODO: remove
  Node *isItemOfParentKvg;
  ParseInfoL pi;
  GraphEditCallbackL callbacks;

  //-- constructors
  Graph();
  explicit Graph(const char* filename);
  Graph(const std::map<std::string, std::string>& dict);
  Graph(std::initializer_list<ItemInitializer> list);
  Graph(const Graph& G);
  ~Graph();
  void clear();
  NodeL& list() { return *this; }

  //-- copy operator
  Graph& operator=(const Graph& G){
    if(isItemOfParentKvg) copy(G,NULL); //this is already a subgraph
    else if(G.isItemOfParentKvg) copy(G, &G.isItemOfParentKvg->container); //copy as subgraph (including the item!)
    else copy(G,NULL); //root graph plain copy
    return *this;
  }
  void copy(const Graph& G, Graph* becomeSubgraphOfContainer);
  
  //-- get items
  Node* getItem(const char *key) const;
  Node* getItem(const char *key1, const char *key2);
  Node* getItem(const StringA &keys);
  Node* operator[](const char *key) { return getItem(key); }
  Node& I(const char *key) { Node *it=getItem(key); CHECK(it,"item '" <<key <<"' does not exist"); return *it; }
  Node* getChild(Node *p1, Node *p2) const; //TODO -> getEdge

  //-- get lists of items (TODO: return NodeL, not referring Graph)
  Graph getItems(const char* key);
  Graph getItemsOfDegree(uint deg);
  Graph getTypedItems(const char* key, const std::type_info& type);
  template<class T> Graph getTypedItems(const char* key=NULL){ return getTypedItems(key, typeid(T)); }
  template<class T> NodeL getDerivedItems();

  //-- get values directly (TODO: remove)
  template<class T> T* getValue(const char *key);
  template<class T> T* getValue(const StringA &keys);
  template<class T> bool getValue(T& x, const char *key) { T* y=getValue<T>(key); if(y) { x=*y; return true; } return false; }
  template<class T> bool getValue(T& x, const StringA &keys) { T* y=getValue<T>(keys); if(y) { x=*y; return true; } return false; }

  //-- get lists of all values of a certain type T (or derived from T)
  template<class T> MT::Array<T*> getTypedValues(const char* key=NULL);
  template<class T> MT::Array<T*> getDerivedValues();
  
  //-- adding items
  template<class T> Node *append(T *x, bool ownsValue);
  template<class T> Node *append(const char* key, T *x, bool ownsValue);
  template<class T> Node *append(const StringA& keys, const NodeL& parents, T *x, bool ownsValue);
//  template<class T> Node *append(const StringA& keys, T *x, bool ownsValue) { return append(keys, NodeL(), x, ownsValue); }
//  template<class T> Node *append(const char *key, T *x, bool ownsValue) { return append({MT::String(key)}, NodeL(), x, ownsValue); }
//  template<class T> Node *append(const char *key1, const char* key2, T *x, bool ownsValue) {  return append({MT::String(key1), MT::String(key2)}, NodeL(), x, ownsValue); }
  Node *append(const uintA& parentIdxs);

  void appendDict(const std::map<std::string, std::string>& dict);

  //-- merging items  //TODO: explain better
  Node *merge(Node* m); //removes m and deletes, if it is a member of This and merged with another Node
  void merge(const NodeL& L){ for(Node *m:L) merge(m); }

  //-- debugging
  bool checkConsistency() const;

  //-- indexing
  uint index(bool subKVG=false, uint start=0); //TODO: make private

  //-- I/O
  void sortByDotOrder();
  ParseInfo& getParseInfo(Node *it);
  
  void read(std::istream& is, bool parseInfo=false);
  void write(std::ostream& os=std::cout, const char *ELEMSEP="\n", const char *delim=NULL) const;
  void writeDot(std::ostream& os, bool withoutHeader=false, bool defaultEdges=false, int nodesOrEdges=0);
};
stdPipes(Graph);

//===========================================================================

NodeL neighbors(Node*);

//===========================================================================

struct GraphEditCallback {
  virtual ~GraphEditCallback(){}
  virtual void cb_new(Node*){}
  virtual void cb_delete(Node*){}
};

//===========================================================================


inline Graph GRAPH(const NodeL& L){ //TODO: remove
  Graph G;
  G.isReferringToItemsOf = (Graph*)(1);
  G.NodeL::operator=(L);
  return G;
}

inline bool ItemComp(Node* const& a, Node* const& b){ //TODO: why?
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
    Node *i = kvg.getItem(key);
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
    Node *i = kvg.getItem(key);
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
