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
extern NodeL& NoNodeL; //this is a pointer to NULL! I use it for optional arguments
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
  Graph& graph(){ Graph *graph=getValue<Graph>(); CHECK(graph,""); return *graph; }

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
stdOutPipe(Node)

//===========================================================================

struct Graph : NodeL {
  struct sKeyValueGraph *s;
  Graph* isReferringToNodesOf; //TODO: remove
  Node *isNodeOfParentGraph;

  ParseInfoL pi;
  GraphEditCallbackL callbacks;


  //-- constructors
  Graph();
  explicit Graph(const char* filename);
  explicit Graph(istream& is);
  Graph(const std::map<std::string, std::string>& dict);
  Graph(std::initializer_list<struct NodeInitializer> list);
  Graph(const Graph& G);
  ~Graph();
  void clear();
  NodeL& list() { return *this; }

  //-- copy operator
  Graph& operator=(const Graph& G){
    if(isNodeOfParentGraph) copy(G,NULL); //this is already a subgraph
    else if(G.isNodeOfParentGraph) copy(G, &G.isNodeOfParentGraph->container); //copy as subgraph (including the item!)
    else copy(G,NULL); //root graph plain copy
    return *this;
  }
  void copy(const Graph& G, Graph* becomeSubgraphOfContainer);
  
  //-- get items
  Node* getNode(const char *key) const;
  Node* getNode(const char *key1, const char *key2) const;
  Node* getNode(const StringA &keys) const;
  Node* operator[](const char *key) const{ return getNode(key); }
  Node& I(const char *key) { Node *it=getNode(key); CHECK(it,"item '" <<key <<"' does not exist"); return *it; }
  Node* getChild(Node *p1, Node *p2) const; //TODO -> getEdge

  //-- get lists of items
  NodeL getNodes(const char* key) const;
  NodeL getNodesOfDegree(uint deg);
  NodeL getTypedNodes(const char* key, const std::type_info& type);
  template<class T> NodeL getTypedNodes(const char* key=NULL){ return getTypedNodes(key, typeid(T)); }
  template<class T> NodeL getDerivedNodes();

  //-- get values directly (TODO: remove V and 'getValue', just get should be enough)
  template<class T> T& get(const char *key) const;
  template<class T> const T& get(const char *key, const T& defaultValue) const;
  template<class T> T& V(const char *key){ T* y=getValue<T>(key); CHECK(y,""); return *y; }
  template<class T> const T& V(const char *key, const T& defaultValue) const{ T* y=getValue<T>(key); if(y) return *y; return defaultValue; }
  template<class T> T* getValue(const char *key) const;
  template<class T> T* getValue(const StringA &keys) const;
  template<class T> bool getValue(T& x, const char *key) { T* y=getValue<T>(key); if(y) { x=*y; return true; } return false; }
  template<class T> bool getValue(T& x, const StringA &keys) { T* y=getValue<T>(keys); if(y) { x=*y; return true; } return false; }

  //-- get lists of all values of a certain type T (or derived from T)
  template<class T> MT::Array<T*> getTypedValues(const char* key=NULL);
  template<class T> MT::Array<T*> getDerivedValues();
  
  //-- adding items
  template<class T> Node *append(T *x, bool ownsValue);
//  template<class T> Node *append(const char* key, T *x, bool ownsValue);
  template<class T> Node *append(const StringA& keys, const NodeL& parents, const T& x);
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
  void writeParseInfo(std::ostream& os);
};
stdPipes(Graph);

//===========================================================================

struct NodeInitializer{
  NodeInitializer(const char* key);
  template<class T> NodeInitializer(const char* key, const T& x);
  template<class T> NodeInitializer(const char* key, const StringA& parents, const T& x);
  Graph G;
  Node *it;
  StringA parents;
};

#define NI(key, val) NodeInitializer(#key, val)
#define NIs(key, val) NodeInitializer(#key, MT::String(#val))

//===========================================================================

NodeL neighbors(Node*);

//===========================================================================

struct GraphEditCallback {
  virtual ~GraphEditCallback(){}
  virtual void cb_new(Node*){}
  virtual void cb_delete(Node*){}
};

//===========================================================================


inline bool NodeComp(Node* const& a, Node* const& b){ //TODO: why?
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
  Graph graph;

  template<class T>
  void set(const char *key, const T &value) {
    Node *i = graph.getNode(key);
    if(i) *i->getValue<T>() = value;
    else graph.append({key}, {}, new T(value), true);
  }

  template<class T>
  bool get(const char *key, T &value) { return graph.getValue(value, key); }

  template<class T>
  T* get(const char *key) { return graph.getValue<T>(key); }

  void clear() { graph.clear(); }

  bool remove(const char *key) {
    delete graph[key];
    Node *i = graph.getNode(key);
    if(!i) return false;
    delete i;
//    // TODO is list() here necessary?
//    graph.list().remove(i->index);
    return true;
  }

  void write(std::ostream &os = std::cout) const {
    os << "params = {" << std::endl;
    graph.write(os, " ");
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
