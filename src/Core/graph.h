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
  const void *value_ptr;
  Graph& container;
  StringA keys;
  NodeL parents;
  NodeL parentOf;
  uint index;

  Node(const std::type_info& _type, void *_value_ptr, Graph& _container);
  Node(const std::type_info& _type, void *_value_ptr, Graph& _container, const StringA& _keys, const NodeL& _parents);
  virtual ~Node();

  //-- get value
  template<class T> bool isOfType() const{ return type==typeid(T); }
  template<class T> T *getValue();    ///< query whether node type is equal to (or derived from) T, return the value if so
  template<class T> const T *getValue() const; ///< as above
  template<class T> T& get(){ T *x=getValue<T>(); CHECK(x, "this node is not of type '" <<typeid(T).name() <<"'"); return *x; }
  template<class T> const T& get() const{ const T *x=getValue<T>(); CHECK(x, "this node is not of type '" <<typeid(T).name() <<"'"); return *x; }
  Graph& graph() { return get<Graph>(); }
  const Graph& graph() const { return get<Graph>(); }
  bool isBoolAndTrue() const{ if(type!=typeid(bool)) return false; return *((bool*)value_ptr) == true; }
  bool isBoolAndFalse() const{ if(type!=typeid(bool)) return false; return *((bool*)value_ptr) == false; }
  bool isGraph() const;//{ return type==typeid(Graph); }

  bool matches(const char *key); ///< return true, if 'key' is in keys
  bool matches(const StringA &query_keys); ///< return true, if all query_keys are in keys

  void write(std::ostream &os) const;

  //-- virtuals implemented by Node_typed
  virtual void copyValue(Node*) {NIY}
  virtual bool hasEqualValue(Node*) {NIY}
  virtual void writeValue(std::ostream &os) const {NIY}
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
  ~Graph();

  void clear();
  NodeL& list() { return *this; }

  //-- copy operator
  Graph& operator=(const Graph& G){  copy(G);  return *this;  }
  void copy(const Graph& G, bool appendInsteadOfClear=false, bool allowCopySubgraphToNonsubgraph=false);
  
  //-- adding nodes
  template<class T> Node *append(const StringA& keys, const NodeL& parents, const T& x); ///<exactly equivalent to calling a Node_typed constructor
  Node *append(const uintA& parentIdxs); ///< add 'vertex tupes' (like edges) where vertices are referred to by integers
  Node *append(const Nod& ni); ///< (internal) append a node initializer
  Node_typed<Graph>* appendSubgraph(const StringA& keys, const NodeL& parents, const Graph& x=NoGraph);
  void appendDict(const std::map<std::string, std::string>& dict);

  //-- basic node retrieval -- users usually use the higher-level wrappers below
  Node* findNode (const StringA& keys=StringA(), bool recurseUp=false, bool recurseDown=false) const;  ///< returns NULL if not found
  NodeL findNodes(const StringA& keys=StringA(), bool recurseUp=false, bool recurseDown=false) const;
  Node* findNodeOfType (const std::type_info& type, const StringA& keys=StringA(), bool recurseUp=false, bool recurseDown=false) const;
  NodeL findNodesOfType(const std::type_info& type, const StringA& keys=StringA(), bool recurseUp=false, bool recurseDown=false) const;

  //-- get nodes
  Node* operator[](const char *key) const{ Node *n = findNode({key}); return n; }//CHECK(n, "key '" <<key <<"' not found"); return n; }
  Node* getNode(const char *key) const{ return findNode({key}, true, false); }
  Node* getNode(const StringA &keys) const{ return findNode(keys, true, false); }
  Node* getEdge(Node *p1, Node *p2) const;

  //-- get lists of nodes
  NodeL getNodes(const char* key) const{ return findNodes({key}); }
  NodeL getNodes(const StringA &keys) const{ return findNodes(keys); }
  NodeL getNodesOfDegree(uint deg);
  template<class T> NodeL getNodesOfType(){ return findNodesOfType(typeid(T)); }
  template<class T> NodeL getNodesOfType(const char* key){ return findNodesOfType(typeid(T), {key}); }

  //-- get values directly
  template<class T> T* find(const char *key)     const { Node *n = findNodeOfType(typeid(T), {key}); if(!n) return NULL;  return n->getValue<T>(); }
  template<class T> T* find(const StringA &keys) const { Node *n = findNodeOfType(typeid(T), keys);  if(!n) return NULL;  return n->getValue<T>(); }
  template<class T> T& get(const char *key) const;
  template<class T> T& get(const StringA &keys) const;
  template<class T> const T& get(const char *key, const T& defaultValue) const;
  template<class T> bool get(T& x, const char *key)     const { Node *n = findNodeOfType(typeid(T), {key}); if(!n) return false;  x=n->get<T>();  return true; }
  template<class T> bool get(T& x, const StringA &keys) const { Node *n = findNodeOfType(typeid(T), keys);  if(!n) return false;  x=n->get<T>();  return true; }

  //-- get lists of all values of a certain type T (or derived from T)
  template<class T> mlr::Array<T*> getValuesOfType(const char* key=NULL);
  
  //-- merging nodes  //TODO: explain better
  Node *merge(Node* m); //removes m and deletes, if it is a member of This and merged with another Node
  void merge(const NodeL& L){ for(Node *m:L) merge(m); }

  //-- debugging
  bool checkConsistency() const;

  //-- I/O
  void sortByDotOrder();
  ParseInfo& getParseInfo(Node *it);
  
  void read(std::istream& is, bool parseInfo=false);
  Node* readNode(std::istream& is, bool verbose=false, bool parseInfo=false, mlr::String prefixedKey=mlr::String()); //used only internally..
  void write(std::ostream& os=std::cout, const char *ELEMSEP="\n", const char *delim=NULL) const;
  void writeDot(std::ostream& os, bool withoutHeader=false, bool defaultEdges=false, int nodesOrEdges=0, int focusIndex=-1);
  void writeHtml(std::ostream& os, std::istream& is);
  void writeParseInfo(std::ostream& os);

private:
  friend struct Node;
  friend struct sGraphView;
  uint index(bool subKVG=false, uint start=0);
};
stdPipes(Graph)

bool operator==(const Graph& A, const Graph& B);

Node_typed<Graph>* newSubGraph(Graph& container, const StringA& keys, const NodeL& parents); ///< creates this as a subgraph-node of container

inline bool Node::isGraph() const{ return type==typeid(Graph); }

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
    if(i) i->get<T>() = value;
    else graph.append({key}, {}, value);
  }

  template<class T>
  bool get(const char *key, T &value) { return graph.get(value, key); }

  template<class T>
  T* get(const char *key) { return graph.find<T>(key); }

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
