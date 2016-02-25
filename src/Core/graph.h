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

#ifndef MLR_graph_h
#define MLR_graph_h

#include "array.h"
#include <map>

struct Node;
template<class T> struct Node_typed;
struct Graph;
struct ParseInfo;
struct GraphEditCallback;
typedef mlr::Array<Node*> NodeL;
typedef mlr::Array<ParseInfo*> ParseInfoL;
typedef mlr::Array<GraphEditCallback*> GraphEditCallbackL;
extern NodeL& NoNodeL; //this is a reference to NULL! I use it for optional arguments
extern Graph& NoGraph; //this is a reference to NULL! I use it for optional arguments

//===========================================================================

struct Node {
  const std::type_info& type;
  Graph& container;
  StringA keys;
  NodeL parents;
  NodeL parentOf;
  uint index;

  Node(const std::type_info& _type, Graph& _container);
  Node(const std::type_info& _type, Graph& _container, const StringA& _keys, const NodeL& _parents);
  virtual ~Node();

  //-- get value
  template<class T> T *getValue();    ///< query whether the Node is of a certain value, return the value if so
  template<class T> const T *getValue() const; ///< as above
  template<class T> T& V(){ T *x=getValue<T>(); CHECK(x, "this node is not of type '" <<typeid(T).name() <<"'"); return *x; }
  template<class T> const T& V() const{ const T *x=getValue<T>(); CHECK(x, "this node is not of type '" <<typeid(T).name() <<"'"); return *x; }
  Graph& graph() const{ Graph *graph=V<Graph*>(); CHECK(graph, "this node is not of type 'Graph'"); return *graph; }
  bool isBoolAndTrue() const{ const bool *value=getValue<bool>(); if(!value) return false; return *value; }
  bool isGraph() const{ return type==typeid(Graph*); }

  bool matches(const char *key); ///< return true, if 'key' is in keys
  bool matches(const StringA &query_keys); ///< return true, if all query_keys are in keys

  void write(std::ostream &os) const;

  //-- virtuals implemented by Node_typed
  virtual bool hasValue() const {NIY}
  virtual void* getValueDirectly() const {NIY}
  virtual void writeValue(std::ostream &os) const {NIY}
  virtual const std::type_info& getValueType() const {NIY}
  virtual void takeoverValue(Node *it) {NIY}
  virtual bool is_derived_from_RootType() const {NIY}
  virtual void copyValue(Node*) {NIY}
  virtual bool hasEqualValue(Node*) {NIY}
  virtual Node* newClone(Graph& container) const {NIY}
};
stdOutPipe(Node)

//===========================================================================

struct Graph : NodeL {
  Node *isNodeOfParentGraph;

  ParseInfoL pi;
  GraphEditCallbackL callbacks;

  //-- constructors
  Graph();                                               ///< empty graph
  explicit Graph(const char* filename);                  ///< read from a file
  explicit Graph(istream& is);                           ///< read from a stream
  Graph(const std::map<std::string, std::string>& dict); ///< useful to represent Python dicts
  Graph(std::initializer_list<struct Nod> list);         ///< initialize, e.g.: {"x", "b", {"a", 3.}, {"b", {"x"}, 5.}, {"c", mlr::String("BLA")} };
  Graph(const Graph& G);                                 ///< copy constructor
//  Graph(Graph& container, const StringA& keys, const NodeL& parents); ///< creates this as a subgraph-node of container
  ~Graph();

  void clear();
  NodeL& list() { return *this; }

  //-- copy operator
  Graph& operator=(const Graph& G){
//    if(isNodeOfParentGraph) copy(G, NULL); //this is already a subgraph
//    else if(G.isNodeOfParentGraph) copy(G, &G.isNodeOfParentGraph->container); //copy as subgraph (including the node!)
//    else copy(G, NULL); //root graph plain copy
    xx_graph_copy(G);
    return *this;
  }
  void xx_graph_copy(const Graph& G, bool appendInsteadOfClear=false);
  
  //-- get nodes
  Node* getNode(const char *key) const;      ///< returns NULL if not found
  Node* getNode(const StringA &keys) const;  ///< returns NULL if not found
  Node* operator[](const char *key) const{ return getNode(key); }
  Node* getEdge(Node *p1, Node *p2) const;
  template<class T> Node* getNodeOfType(const char *key) const;

  //-- get lists of nodes
  NodeL getNodes(const char* key) const;
  NodeL getNodes(const StringA &keys) const;
  NodeL getNodesOfDegree(uint deg);
  NodeL getNodesOfType(const char* key, const std::type_info& type);
  template<class T> NodeL getNodesOfType(const char* key=NULL){ return getNodesOfType(key, typeid(T)); }
  template<class T> NodeL getDerivedNodes();

  //-- get values directly
  template<class T> T* getValue(const char *key)     const { Node *n = getNode(key);   if(!n) return NULL;  return n->getValue<T>(); }
  template<class T> T* getValue(const StringA &keys) const { Node *n = getNode(keys);  if(!n) return NULL;  return n->getValue<T>(); }
  template<class T> T& get(const char *key) const;
  template<class T> T& get(const StringA &keys) const;
  template<class T> const T& get(const char *key, const T& defaultValue) const;
  template<class T> bool get(T& x, const char *key)     const { T* y=getValue<T>(key);  if(!y) return false;  x=*y;  return true; }
  template<class T> bool get(T& x, const StringA &keys) const { T* y=getValue<T>(keys); if(!y) return false;  x=*y;  return true; }

  //-- get lists of all values of a certain type T (or derived from T)
  template<class T> mlr::Array<T*> getValuesOfType(const char* key=NULL);
  template<class T> mlr::Array<T*> getDerivedValues();
  
  //-- adding nodes
//  template<class T> Node *append(T *x, bool ownsValue);
  template<class T> Node *append(const StringA& keys, const NodeL& parents, const T& x);
//  template<class T> Node *append(const StringA& keys, const NodeL& parents, T *x, bool ownsValue);
  Node *append(const uintA& parentIdxs);
  Node *append(const Nod& ni); ///< (internal) append a node initializer
  void appendDict(const std::map<std::string, std::string>& dict);

  //-- merging nodes  //TODO: explain better
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
  Node* readNode(std::istream& is, bool verbose=false, bool parseInfo=false, mlr::String prefixedKey=mlr::String()); //used only internally..
  void write(std::ostream& os=std::cout, const char *ELEMSEP="\n", const char *delim=NULL) const;
  void writeDot(std::ostream& os, bool withoutHeader=false, bool defaultEdges=false, int nodesOrEdges=0, int focusIndex=-1);
  void writeHtml(std::ostream& os, std::istream& is);
  void writeParseInfo(std::ostream& os);
};
stdPipes(Graph);

bool operator==(const Graph& A, const Graph& B);

Node_typed<Graph*>* newSubGraph(Graph& container, const StringA& keys, const NodeL& parents); ///< creates this as a subgraph-node of container

//===========================================================================

/// This is a Node initializer, specifically for Graph(std::initializer_list<struct Nod> list); and the operator<< below
struct Nod{
  Nod(const char* key);
  template<class T> Nod(const char* key, const T& x);
  template<class T> Nod(const char* key, const StringA& parents, const T& x);
  Graph G;
  Node *n;
  StringA parents;
};

/// pipe node initializers into a graph (to append nodes)
inline Graph& operator<<(Graph& G, const Nod& n){ G.append(n); return G; }

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

#include "graph.tpp"

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
    else graph.append({key}, {}, value);
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
