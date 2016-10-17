/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


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
struct RenderingInfo;
struct GraphEditCallback;
typedef mlr::Array<Node*> NodeL;
typedef mlr::Array<ParseInfo*> ParseInfoL;
typedef mlr::Array<RenderingInfo*> RenderingInfoL;
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
  virtual void copyValueInto(void*) const {NIY}
  virtual Node* newClone(Graph& container) const {NIY}
};
stdOutPipe(Node)

//===========================================================================

struct Graph : NodeL {
  Node *isNodeOfGraph; // rename: isNodeOfGraph

  ParseInfoL pi;
  RenderingInfoL ri;
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
  
  //-- adding nodes (TODO:rename to newNode, newEdge)
  template<class T> Node_typed<T>* newNode(const StringA& keys, const NodeL& parents, const T& x); ///<exactly equivalent to calling a Node_typed constructor
  template<class T> Node_typed<T>* newNode(const T& x); ///<exactly equivalent to calling a Node_typed constructor
  Node_typed<int>* newNode(const uintA& parentIdxs); ///< add 'vertex tupes' (like edges) where vertices are referred to by integers
  Graph& newNode(const Nod& ni); ///< (internal) append a node initializer
  Node_typed<Graph>* newSubgraph(const StringA& keys, const NodeL& parents, const Graph& x=NoGraph);
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
  Node* getEdge(const NodeL& parents) const;

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

  //-- hierarchical finding: up and down in the graph hierarchy
  const Graph* getRootGraph() const;
  bool isChildOfGraph(const Graph& G) const;

  //-- debugging
  bool checkConsistency() const;

  //-- I/O
  void sortByDotOrder();
  ParseInfo& getParseInfo(Node *n);
  bool hasRenderingInfo(Node *n){ return n->index<ri.N; }
  RenderingInfo& getRenderingInfo(Node *n);

  void read(std::istream& is, bool parseInfo=false);
  Node* readNode(std::istream& is, bool verbose=false, bool parseInfo=false, mlr::String prefixedKey=mlr::String()); //used only internally..
  void write(std::ostream& os=std::cout, const char *ELEMSEP="\n", const char *delim=NULL) const;
  void writeDot(std::ostream& os, bool withoutHeader=false, bool defaultEdges=false, int nodesOrEdges=0, int focusIndex=-1);
  void writeHtml(std::ostream& os, std::istream& is);
  void writeParseInfo(std::ostream& os);

  //private:
  friend struct Node;
  friend struct sGraphView;
  uint index(bool subKVG=false, uint start=0);
};
stdPipes(Graph)

bool operator==(const Graph& A, const Graph& B);

Node_typed<Graph>* newSubGraph(Graph& container, const StringA& keys, const NodeL& parents); ///< creates this as a subgraph-node of container

inline bool Node::isGraph() const{ return type==typeid(Graph); }

//===========================================================================

struct GraphEditCallback {
  virtual ~GraphEditCallback(){}
  virtual void cb_new(Node*){}
  virtual void cb_delete(Node*){}
  virtual void cb_graphDestruct(){}
};

//===========================================================================

template<class T>
struct ArrayG : mlr::Array<T>, GraphEditCallback {
  Graph& G;

  ArrayG(Graph& _G):G(_G){ this->memMove=true;  this->resize(G.N);  G.callbacks.append(this); }
  ~ArrayG(){ G.callbacks.removeValue(this); }

  T& operator()(Node *n) const { return this->elem(n->index); }

  virtual void cb_new(Node *n){ this->insert(n->index, T()); }
  virtual void cb_delete(Node *n){ this->remove(n->index); }
};

//===========================================================================


#define GRAPH(str) \
  Graph(mlr::String(str).stream())

#ifndef _NUMARGS
#  define _NUMARGS2(X,X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#  define _NUMARGS(...) _NUMARGS2(0, __VA_ARGS__ ,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)
#endif

#define _GRA_2(key, value) .append(Nod(#key, value))
#define _GRA_4(key, value, ...) .append(Nod(#key, value)) _GRA_2(__VA_ARGS__)
#define _GRA_6(key, value, ...) .append(Nod(#key, value)) _GRA_4(__VA_ARGS__)
#define _GRA_8(key, value, ...) .append(Nod(#key, value)) _GRA_6(__VA_ARGS__)
#define _GRA_10(key, value, ...) .append(Nod(#key, value)) _GRA_8(__VA_ARGS__)
#define _GRA_12(key, value, ...) .append(Nod(#key, value)) _GRA_10(__VA_ARGS__)
#define _GRA_14(key, value, ...) .append(Nod(#key, value)) _GRA_12(__VA_ARGS__)
#define _GRA_16(key, value, ...) .append(Nod(#key, value)) _GRA_14(__VA_ARGS__)
#define _GRA_18(key, value, ...) .append(Nod(#key, value)) _GRA_16(__VA_ARGS__)
#define _GRA_N2(N, ...) _GRA_ ## N(__VA_ARGS__)
#define _GRA_N1(N, ...) _GRA_N2(N, __VA_ARGS__) //this forces that _NUMARGS(...) is expanded to a number N
#define GRA(...)  ( Graph() _GRA_N1(_NUMARGS(__VA_ARGS__), __VA_ARGS__) )

//===========================================================================

/// This is a Node initializer, specifically for Graph(std::initializer_list<struct Nod> list); and the operator<< below
struct Nod{
  Nod(const char* key);
  Nod(const char* key, const char* stringValue);
  template<class T> Nod(const char* key, const T& x);
  template<class T> Nod(const char* key, const StringA& parents, const T& x);
  Graph G;
  Node *n;
  StringA parents;
};

/// pipe node initializers into a graph (to append nodes)
inline Graph& operator<<(Graph& G, const Nod& n){ G.newNode(n); return G; }

//===========================================================================

NodeL neighbors(Node*);

//===========================================================================
//
// annotations to a node for rendering; esp dot
//

struct RenderingInfo{
  Node *node;
  mlr::String dotstyle;
  RenderingInfo():node(NULL){}
  void write(ostream& os) const{ os <<dotstyle; }
};
stdOutPipe(RenderingInfo)

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
    else graph.newNode({key}, {}, value);
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
