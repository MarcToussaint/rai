/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "array.h"
#include "util.h"

#include <math.h>
#include <map>
#include <memory>

//===========================================================================

namespace rai {
struct Node;
template<class T> struct Node_typed;
template<class T> struct ArrayG;
struct Graph;
struct ParseInfo;
struct RenderingInfo;
struct GraphEditCallback;
struct BracketOp;
typedef Array<Node*> NodeL;
typedef Array<GraphEditCallback*> GraphEditCallbackL;
}
extern rai::NodeL& NoNodeL; //this is a reference to nullptr! (for optional arguments)
extern rai::Graph& NoGraph; //this is a reference to nullptr! (for optional arguments)

//===========================================================================

namespace rai {
struct Node {
  const std::type_info& type;
  Graph& container;
  String key;
  NodeL parents;
  NodeL children;
  uint numChildren=0;
  uint index;

  Node(const std::type_info& _type, Graph& _container, const char* _key);
  virtual ~Node();

  Node* addParent(Node* p, bool prepend=false);
  Node* setParents(const NodeL& P);
  void removeParent(Node* p);
  void swapParent(uint i, Node* p);

  //-- get value
  template<class T> bool is() const { return type==typeid(T); }
  template<class T> T& as() { T* x=getValue<T>(); CHECK(x, "this node '" <<*this <<"' is not of type '" <<typeid(T).name() <<"' but type '" <<type.name() <<"'"); return *x; }
  template<class T> const T& as() const { const T* x=getValue<T>(); CHECK(x, "this node '" <<*this <<"'is not of type '" <<typeid(T).name() <<"' but type '" <<type.name() <<"'"); return *x; }
  template<class T> T& asHard() { return ((Node_typed<T>*)(this))->value; }

  template<class T> T* getValue();    ///< query whether node type is equal to (or derived from) T, return the value if so
  template<class T> const T* getValue() const; ///< as above
  template<class T> bool getFromDouble(T& x) const; ///< return value = false means parsing object of type T from the double failed
  template<class T> bool getFromString(T& x) const; ///< return value = false means parsing object of type T from the string failed
  template<class T> bool getFromArr(T& x) const; ///< return value = false means parsing object of type T from the arr failed
  bool isBoolAndTrue() const { if(type!=typeid(bool)) return false; return *getValue<bool>() == true; }
  bool isBoolAndFalse() const { if(type!=typeid(bool)) return false; return *getValue<bool>() == false; }

  //-- get sub-value assuming this is a graph
  Graph& graph() { return as<Graph>(); }
  const Graph& graph() const { return as<Graph>(); }

  void getSubtree(NodeL& N) const {
    for(Node* child:children) { N.append(child); child->getSubtree(N); }
  }

  void write(std::ostream& os, int indent=-1, bool yamlMode=false, bool binary=false) const;

  //-- virtuals implemented by Node_typed
  virtual void copyValue(Node*) {NIY}
  virtual bool hasEqualValue(Node*) {NIY}
  virtual void writeValue(std::ostream& os) const {NIY}
  virtual Node* newClone(Graph& container) const {NIY}
};
stdOutPipe(Node)

} //namespace

//===========================================================================

namespace rai {
struct Graph : NodeL {
  Node* isNodeOfGraph; ///< THIS is a subgraph of another graph; isNodeOfGraph points to the node that equals THIS graph
  bool isIndexed=true;
  bool isDoubleLinked=true;

  ArrayG<ParseInfo>* pi;     ///< optional annotation of nodes: when detailed file parsing is enabled
  ArrayG<RenderingInfo>* ri; ///< optional annotation of nodes: dot style commands

  //-- constructors
  Graph();                                               ///< empty graph
  explicit Graph(const char* filename, bool parseInfo=false);         ///< read from a file
  explicit Graph(istream& is);                           ///< read from a stream
  Graph(const std::map<std::string, std::string>& dict); ///< useful to represent Python dicts
  Graph(std::initializer_list<struct NodeInitializer> list);         ///< initialize, e.g.: {"x", "b", {"a", 3.}, {"b", {"x"}, 5.}, {"c", rai::String("BLA")} };
  Graph(const Graph& G);                                 ///< copy constructor
  ~Graph();
  bool operator!() const;                                ///< check if NoGraph

  void clear();
  NodeL& list() { return *this; }

  //-- copy operator
  Graph& operator=(const Graph& G) {  copy(G, false, true);  return *this;  }
  void copy(const Graph& G, bool appendInsteadOfClear=false, bool enforceCopySubgraphToNonsubgraph=false);

  //-- adding nodes
  template<class T> Node_typed<T>* add(const char* key);
  template<class T> Node_typed<T>* add(const char* key, const T& x);
  template<class T> Node_typed<T&>* addRef(const char* key, const T& x);

  //Node_typed<int>* add(const uintA& parentIdxs); ///< add 'vertex tupes' (like edges) where vertices are referred to by integers
  Graph& addSubgraph(const char* key=NULL, const NodeL& parents= {});
  void appendDict(const std::map<std::string, std::string>& dict);
  Graph& addInit(const NodeInitializer& ni); ///< (internal) append a node initializer

  //-- deleting nodes
  void delNode(Node* n) { CHECK(n, "can't delete NULL"); delete n; }

  //-- basic node retrieval -- users should use the higher-level wrappers below
  Node* findNode(const char* key, bool recurseUp=false, bool recurseDown=false) const;   ///< returns nullptr if not found
  NodeL findNodes(const char* key, bool recurseUp=false, bool recurseDown=false) const;
  Node* findNodeOfType(const std::type_info& type, const char* key=0, bool recurseUp=false, bool recurseDown=false) const;
  NodeL findNodesOfType(const std::type_info& type, const char* key=0, bool recurseUp=false, bool recurseDown=false) const;
  NodeL findGraphNodesWithTag(const char* tag) const;

  //
  template<class T> Node* set(const char* key, const T& x){ Node* n = findNodeOfType(typeid(T), key); if(n) n->as<T>()=x; else n=add<T>(key, x); return n; }
  Node* set(Node* _n){ Node* n = findNodeOfType(_n->type, _n->key); if(n) n->copyValue(_n); else n=_n->newClone(*this); return n; }

  //-- get nodes
  BracketOp operator[](const char* key); ///< returns nullptr if not found
  Node* getNode(const char* key) const { return findNode(key); } ///< returns nullptr if not found
  Node* getEdge(Node* p1, Node* p2) const;
  Node* getEdge(const NodeL& parents) const;

  //-- get lists of nodes
  NodeL getNodes(const char* key) const { return findNodes(key); }
  NodeL getNodesWithTag(const char* key) const { return findGraphNodesWithTag(key); }
  NodeL getNodesOfDegree(uint deg);
  template<class T> NodeL getNodesOfType() { return findNodesOfType(typeid(T), {}); }
  template<class T> NodeL getNodesOfType(const char* key) { return findNodesOfType(typeid(T), key); }
  NodeL getAllNodesRecursively() const;

  //-- get values directly
  template<class T> T* find(const char* key) const { Node* n = findNodeOfType(typeid(T), key); if(!n) return nullptr;  return n->getValue<T>(); }
  template<class T> T& get(const char* key) const;
  template<class T> const T& get(const char* key, const T& defaultValue) const;
  template<class T> bool get(T& x, const char* key) const;
  template<class T> T& getNew(const char* key);

  //-- get lists of all values of a certain type T (or derived from T)
  template<class T> rai::Array<T*> getValuesOfType(const char* key=nullptr);

  //-- editing nodes
  Node* edit(Node* ed); ///< ed describes how another node should be edited; ed is removed after editing is done
  void edit(const NodeL& L) { for(Node* ed:L) edit(ed); }
  bool checkUniqueKeys(bool makeUnique=false);
  void collapse(Node* a, Node* b);

  //-- hierarchical finding: up and down in the graph hierarchy
  const Graph* getRootGraph() const;
  bool isChildOfGraph(const Graph& G) const;

  //-- debugging
  bool checkConsistency() const;

  //-- I/O
  void sortByDotOrder();
  ParseInfo& getParseInfo(Node* n);
  bool hasRenderingInfo(Node* n) { return ri; }
  RenderingInfo& getRenderingInfo(Node* n);

  void read(std::istream& is, bool parseInfo=false);
  Node* readNode(std::istream& is, bool verbose, bool parseInfo); //used only internally..
  void readJson(std::istream& is);
  void writeJson(std::istream& is);
  void write(std::ostream& os=cout, const char* ELEMSEP="\n", const char* BRACKETS=0, int indent=-1, bool yamlMode=false, bool binary=false) const;
  void writeDot(std::ostream& os, bool withoutHeader=false, bool defaultEdges=false, int nodesOrEdges=0, int focusIndex=-1, bool subGraphsAsNodes=false);
  void writeHtml(std::ostream& os, std::istream& is);
  void writeYaml(std::ostream& os) const;
  void writeParseInfo(std::ostream& os);

  void displayDot(Node* highlight=nullptr);

  //private:
  friend struct Node;
  uint index(bool subKVG=false, uint start=0);

};

bool operator==(const Graph& A, const Graph& B);
stdPipes(Graph)

//===========================================================================

struct GraphEditCallback {
  virtual ~GraphEditCallback() {}
  virtual void cb_new(Node*) {}
  virtual void cb_delete(Node*) {}
  virtual void cb_graphDestruct() {}
};

//===========================================================================

/// To associate additional objects with each node, this simple array stores such
/// objects, resizes automatically and is accessible by node pointer
template<class T>
struct ArrayG : rai::Array<T*>, GraphEditCallback {
  //why a list: the cb_new/delete call insert/remove, which requires memMove
  rai::Graph& G;
  ArrayG(Graph& _G):G(_G) {
    this->memMove=true;
    this->resize(G.N+1).setZero();
  }
  ~ArrayG() {
    for(T* x:*this) if(x) { delete x; x=nullptr; }
    this->clear();
  }
  T& nodeelem(Node* n) {
//    CHECK_EQ(this->N, G.N+1, "");
    while(this->N < G.N+1) this->append(0);
    T* x = 0;
    if(n) x = this->elem(n->index+1);
    else x = this->elem(0);
    if(!x) {
      x = new T(); //...assigned here
      if(n) this->elem(n->index+1) = x;
      else this->elem(0) = x;
    }
    return *x;
  }
  virtual void cb_new(Node* n) { this->insert(n->index+1, (T*)nullptr); }
  virtual void cb_delete(Node* n) { T*& x = this->elem(n->index+1); if(x) { delete x; x=nullptr; } this->remove(n->index+1); }
};

} //namespace

//===========================================================================

#define GRAPH(str) \
  rai::Graph(rai::String(str).stream())

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

namespace rai {
/// This is a Node initializer, specifically for Graph(std::initializer_list<struct Nod> list); and the operator<< below
/// not to be used otherwise
struct NodeInitializer {
  NodeInitializer(const char* key);
  NodeInitializer(const char* key, const char* stringValue);
  template<class T> NodeInitializer(const char* key, const T& x);
  template<class T> NodeInitializer(const char* key, const StringA& parents, const T& x);
  Graph G;
  Node* n;
  StringA parents;
};

/// pipe node initializers into a graph (to append nodes)
inline Graph& operator<<(Graph& G, const NodeInitializer& n) { G.addInit(n); return G; }

//===========================================================================

struct BracketOp {
  Graph& G;
  const char* key;
  Node *n;
  template<class T> void operator=(const T& x){
    if(!n) n = G.add<T>(key, x);
    else n->as<T>() = x;
  }
  Node* operator->() { return n; }
  operator Node*() { return n; }
  //T& operator->() { return *p; }
};

//===========================================================================
//
// algorithms

NodeL neighbors(Node*);

int distance(NodeL A, NodeL B);

template<class VertexType>
rai::Array<VertexType*> getNeighbors(VertexType* v) {
  rai::Array<VertexType*> N;
  for(rai::Node* edge:v->children) {
    for(rai::Node* n:edge->parents) if(n!=v) N.setAppend(dynamic_cast<VertexType*>(n));
  }
  return N;
}

template<class EdgeType>
rai::Array<EdgeType*> getEdges(Node* v) {
  rai::Array<EdgeType*> E(v->children.N);
  for(uint i=0; i<E.N; i++) E.elem(i) = dynamic_cast<EdgeType*>(v->children.elem(i));
  return E;
}

//===========================================================================

/// annotations to a node for rendering; esp dot
struct RenderingInfo {
  rai::String dotstyle;
  bool skip;
  RenderingInfo() : skip(false) {}
  void write(ostream& os) const { os <<dotstyle; }
};
stdOutPipe(RenderingInfo)

//===========================================================================

/// global registry of parameters (taken from cmd line or file) as a singleton graph
Mutex::TypedToken<rai::Graph> params();
void initParameters(int _argc, char* _argv[], bool forceReload=false, bool verbose=true);

//===========================================================================

// registering a type that can parse io streams into a Node --
// using this mechanism, a Graph can parse any type from files, when types
// are registered
template<class T>
struct Type_typed_readable:Type_typed<T> {
  virtual Node* readIntoNewNode(Graph& container, std::istream& is) const { Node_typed<T>* n = container.add<T>(T(0)); is >>n->value; return n; }
};

typedef rai::Array<std::shared_ptr<Type>> TypeInfoL;

//===========================================================================
//===========================================================================
//
// definition of template methods - could move this to graph.ipp
//
//===========================================================================
//===========================================================================

//===========================================================================
//
//  typed Node
//

template<class T>
struct Node_typed : Node {
  T value;

  Node_typed() : value(nullptr) { HALT("shouldn't be called, right? You always want to append to a container"); }

  Node_typed(Graph& container, const char* key)
    : Node(typeid(T), container, key), value() {
    if(is<Graph>()) graph().isNodeOfGraph = this; //this is the only place where isNodeOfGraph is set
  }

  Node_typed(Graph& container, const char* key, const T& _value)
    : Node(typeid(T), container, key), value(_value) {
    if(is<Graph>()) graph().isNodeOfGraph = this; //this is the only place where isNodeOfGraph is set
  }

  virtual ~Node_typed() {
  }

  virtual void copyValue(Node* it) {
    Node_typed<T>* itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt, "can't assign to wrong type");
    value = itt->value;
  }

  virtual bool hasEqualValue(Node* it) {
    Node_typed<T>* itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt, "can't compare to wrong type");
    return value == itt->value;
  }

  virtual void writeValue(std::ostream& os) const {
    if(typeid(T)==typeid(NodeL)) os <<getValue<NodeL>()->modList();
    else os <<value;
  }

  virtual const std::type_info& getValueType() const {
    return typeid(T);
  }

  virtual Node* newClone(Graph& container) const {
    if(is<Graph>()) {
      Graph& g = container.addSubgraph(key, parents);
      g.copy(graph());
      return g.isNodeOfGraph;
    }
    return container.add<T>(key,  value)->setParents(parents);
  }
};
} //namespace

//===========================================================================
//
// Node & Graph template methods
//

namespace rai {
template<class T> T* Node::getValue() {
  Node_typed<T>* typed = dynamic_cast<Node_typed<T>*>(this);
  if(!typed) return nullptr;
  return &typed->value;
}

template<class T> const T* Node::getValue() const {
  const Node_typed<T>* typed = dynamic_cast<const Node_typed<T>*>(this);
  if(!typed) return nullptr;
  return &typed->value;
}

template<class T> bool Node::getFromDouble(T& x) const {
  if(!is<double>()) return false;
  double y = as<double>();
  if(typeid(T)==typeid(int)) {
    CHECK(!modf(y, &y), "numerical parameter " <<key <<" should be integer");
    *((int*)&x)=(int)y;
    return true;
  }
  if(typeid(T)==typeid(uint)) {
    CHECK(!modf(y, &y), "numerical parameter " <<key <<" should be integer");
    *((uint*)&x)=(uint)y;
    return true;
  }
  if(typeid(T)==typeid(bool)) {
    CHECK(y==0. || y==1., "numerical parameter " <<key <<" should be boolean");
    *((bool*)&x)=(y==1.);
    return true;
  }
  return false;
}

template<class T> bool Node::getFromString(T& x) const {
  if(!is<rai::String>()) return false;
  rai::String str = as<rai::String>();
  str.resetIstream() >>x;
  if(str.stream().good()) return true;
  return false;
}

template<class T> bool Node::getFromArr(T& x) const {
  if(!is<arr>()) return false;
  arr z = as<arr>();
  x.set(z);
  return true;
}

template<class T> NodeInitializer::NodeInitializer(const char* key, const T& x) {
  n = G.add<T>(key, x);
}

template<class T> NodeInitializer::NodeInitializer(const char* key, const StringA& parents, const T& x)
  : parents(parents) {
  n = G.add<T>(key, x);
}

inline BracketOp Graph::operator[](const char* key) {
  return BracketOp{*this, key, findNode(key)};
}

template<class T> T& Graph::get(const char* key) const {
  Node* n = findNodeOfType(typeid(T), key);
  if(!n) HALT("no node of type '" <<typeid(T).name() <<"' with key '"<< key<< "' found");
  return n->as<T>();
}

template<class T> T& Graph::getNew(const char* key) {
  Node* n = findNodeOfType(typeid(T), key);
  if(!n) n = new Node_typed<T>(*this, key);
  return n->as<T>();
}

template<class T> const T& Graph::get(const char* key, const T& defaultValue) const {
  Node* n = findNodeOfType(typeid(T), key);
  if(!n) return defaultValue;
  return n->as<T>();
}

template<class T> bool Graph::get(T& x, const char* key) const {
  Node* n = findNodeOfType(typeid(T), key);
  if(n) { x=n->as<T>();  return true; }

  //auto type conversions
  n = findNodeOfType(typeid(double), key);
  if(n) return n->getFromDouble<T>(x);

  n = findNodeOfType(typeid(rai::String), key);
  if(n) return n->getFromString<T>(x);

  return false;
}

template<class T> rai::Array<T*> Graph::getValuesOfType(const char* key) {
  NodeL nodes;
  if(!key) nodes = findNodesOfType(typeid(T), NULL);
  else nodes = findNodesOfType(typeid(T), key);
  rai::Array<T*> ret;
  for(Node* n: nodes) ret.append(n->getValue<T>());
  return ret;
}

template<class T> Node_typed<T>* Graph::add(const char* key) {
  //if(typeid(bool)==typeid(T)) return new Node_typed<T>(*this, key, true); //initialized boolian
  return new Node_typed<T>(*this, key);
}

template<class T> Node_typed<T>* Graph::add(const char* key, const T& x) {
  return new Node_typed<T>(*this, key, x);
}

template<class T> Node_typed<T&>* Graph::addRef(const char* key, const T& x) {
  return new Node_typed<T&>(*this, key, (T&)x);
}

}//namespace
