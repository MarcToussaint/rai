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
#include <bits/shared_ptr.h>
#include <memory>

struct Node;
template<class T> struct Node_typed;
template<class T> struct ArrayG;
struct Graph;
struct ParseInfo;
struct RenderingInfo;
struct GraphEditCallback;
typedef mlr::Array<Node*> NodeL;
typedef mlr::Array<GraphEditCallback*> GraphEditCallbackL;
extern NodeL& NoNodeL; //this is a reference to NULL! (for optional arguments)
extern Graph& NoGraph; //this is a reference to NULL! (for optional arguments)

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
  template<class T> std::shared_ptr<T> getPtr() const;  ///< query whether node type is equal to (or derived from) shared_ptr<T>, return the shared_ptr if so
  template<class T> T& get(){ T *x=getValue<T>(); CHECK(x, "this node is not of type '" <<typeid(T).name() <<"' but type '" <<type.name() <<"'"); return *x; }
  template<class T> const T& get() const{ const T *x=getValue<T>(); CHECK(x, "this node is not of type '" <<typeid(T).name() <<"' but type '" <<type.name() <<"'"); return *x; }
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
  Node *isNodeOfGraph; ///< THIS is a subgraph of another graph; isNodeOfGraph points to the node that equals THIS graph

  GraphEditCallbackL callbacks; ///< list of callbacks that are informed about creation and destruction of nodes

  ArrayG<ParseInfo> *pi;     ///< optional annotation of nodes: when detailed file parsing is enabled
//  ArrayG<RenderingInfo> *ri; ///< optional annotation of nodes: dot style commands
  ArrayG<RenderingInfo> *ri;
//  mlr::Array<RenderingInfo> ri; ///< optional annotation of nodes: dot style commands

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
  void copy(const Graph& G, bool appendInsteadOfClear=false, bool enforceCopySubgraphToNonsubgraph=false);
  
  //-- adding nodes
  template<class T> Node_typed<T>* newNode(const StringA& keys, const NodeL& parents, const T& x); ///<exactly equivalent to calling a Node_typed constructor
  template<class T> Node_typed<T>* newNode(const StringA& keys, const NodeL& parents); ///<exactly equivalent to calling a Node_typed constructor
  template<class T> Node_typed<T>* newNode(const T& x); ///<exactly equivalent to calling a Node_typed constructor
  Node_typed<int>* newNode(const uintA& parentIdxs); ///< add 'vertex tupes' (like edges) where vertices are referred to by integers
  Graph& newNode(const Nod& ni); ///< (internal) append a node initializer
  Node_typed<Graph>* newSubgraph(const StringA& keys, const NodeL& parents, const Graph& x=NoGraph);
  void appendDict(const std::map<std::string, std::string>& dict);

  //-- deleting nodes
  void delNode(Node *n) { delete n; }

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
  
  //-- editing nodes
  Node *edit(Node *ed); ///< ed describes how another node should be edited; ed is removed after editing is done
  void edit(const NodeL& L){ for(Node *ed:L) edit(ed); }

  //-- hierarchical finding: up and down in the graph hierarchy
  const Graph* getRootGraph() const;
  bool isChildOfGraph(const Graph& G) const;

  //-- debugging
  bool checkConsistency() const;

  //-- I/O
  void sortByDotOrder();
  ParseInfo& getParseInfo(Node *n);
  bool hasRenderingInfo(Node *n){ return ri; }
  RenderingInfo& getRenderingInfo(Node *n);

  void read(std::istream& is, bool parseInfo=false);
  Node* readNode(std::istream& is, bool verbose=false, bool parseInfo=false, mlr::String prefixedKey=mlr::String()); //used only internally..
  void write(std::ostream& os=std::cout, const char *ELEMSEP="\n", const char *delim=NULL) const;
  void writeDot(std::ostream& os, bool withoutHeader=false, bool defaultEdges=false, int nodesOrEdges=0, int focusIndex=-1);
  void writeHtml(std::ostream& os, std::istream& is);
  void writeParseInfo(std::ostream& os);

  void displayDot(Node *highlight=NULL);

  //private:
  friend struct Node;
  friend struct sGraphView;
  uint index(bool subKVG=false, uint start=0);
};
stdPipes(Graph)

bool operator==(const Graph& A, const Graph& B);

inline bool Node::isGraph() const{ return type==typeid(Graph); }

//===========================================================================

struct GraphEditCallback {
  virtual ~GraphEditCallback(){}
  virtual void cb_new(Node*){}
  virtual void cb_delete(Node*){}
  virtual void cb_graphDestruct(){}
};


//===========================================================================

/// To associate additional objects with each node, this simple array stores such
/// objects, resizes automatically and is accessible by node pointer
template<class T>
struct ArrayG : mlr::Array<T*>, GraphEditCallback {
  //why a list: the cb_new/delete call insert/remove, which requires memMove
  Graph& G;
  ArrayG(Graph& _G):G(_G){
    this->memMove=true;
    this->resize(G.N+1).setZero();
    G.callbacks.append(this);
  }
  ~ArrayG(){
    G.callbacks.removeValue(this);
    for(T* x:*this) if(x){ delete x; x=NULL; }
    this->clear();
  }
  T& operator()(Node *n){
    CHECK_EQ(this->N, G.N+1,"");
//    if(this->N != G.N+1) listResizeCopy(*this, G.N+1); //redundant, given the callback mechanisms...
    T* &x = (!n? this->elem(0) : this->elem(n->index+1)); //x is a reference!
    if(!x) x = new T(); //...assigned here
    return *x;
  }
  virtual void cb_new(Node *n){ this->insert(n->index+1, (T*)NULL); }
  virtual void cb_delete(Node *n){ T* &x = this->elem(n->index+1); if(x){ delete x; x=NULL; } this->remove(n->index+1); }
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
/// not to be used otherwise
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

/// annotations to a node for rendering; esp dot
struct RenderingInfo{
  mlr::String dotstyle;
  bool skip;
  RenderingInfo() : skip(false){}
  void write(ostream& os) const{ os <<dotstyle; }
};
stdOutPipe(RenderingInfo)

//===========================================================================

/// global registry of anything using a singleton Graph
extern Singleton<Graph> registry;

//===========================================================================

// registering a type that can parse io streams into a Node --
// using this mechanism, a Graph can parse any type from files, when types
// are registered
template<class T>
struct Type_typed_readable:Type_typed<T> {
  virtual Node* readIntoNewNode(Graph& container, std::istream& is) const { Node_typed<T> *n = container.newNode<T>(T()); is >>n->value; return n; }
};

typedef mlr::Array<std::shared_ptr<Type> > TypeInfoL;

//===========================================================================
//===========================================================================
//
// definition of template methods
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

  Node_typed():value(NULL) { HALT("shouldn't be called, right? You always want to append to a container"); }

  Node_typed(Graph& container, const T& _value)
    : Node(typeid(T), &this->value, container), value(_value) {
    if(isGraph()) graph().isNodeOfGraph = this; //this is the only place where isNodeOfGraph is set
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_new(this);
  }

  Node_typed(Graph& container, const StringA& keys, const NodeL& parents)
    : Node(typeid(T), &this->value, container, keys, parents), value() {
    if(isGraph()) graph().isNodeOfGraph = this; //this is the only place where isNodeOfGraph is set
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_new(this);
  }

  Node_typed(Graph& container, const StringA& keys, const NodeL& parents, const T& _value)
    : Node(typeid(T), &this->value, container, keys, parents), value(_value) {
    if(isGraph()) graph().isNodeOfGraph = this; //this is the only place where isNodeOfGraph is set
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_new(this);
  }

  virtual ~Node_typed(){
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_delete(this);
  }

  virtual void copyValue(Node *it) {
    Node_typed<T> *itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
    value = itt->value;
  }

  virtual bool hasEqualValue(Node *it) {
    Node_typed<T> *itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt,"can't compare to wrong type");
    return value == itt->value;
  }

  virtual void writeValue(std::ostream &os) const {
    if(typeid(T)==typeid(NodeL)) listWrite(*getValue<NodeL>(), os, " ");
    else os <<value;
  }
  
  virtual void copyValueInto(void *value_ptr) const {
    *((T*)value_ptr) = value;
  }

  virtual const std::type_info& getValueType() const {
    return typeid(T);
  }
  
  virtual Node* newClone(Graph& container) const {
    if(isGraph()){
      Node_typed<Graph> *n = container.newSubgraph(keys, parents);
      n->value.copy(graph());
      return n;
    }
    return container.newNode<T>(keys, parents, value);
  }
};

//===========================================================================
//
// Node & Graph template methods
//

template<class T> T* Node::getValue() {
  Node_typed<T>* typed = dynamic_cast<Node_typed<T>*>(this);
  if(!typed) return NULL;
  return &typed->value;
}

template<class T> const T* Node::getValue() const {
  const Node_typed<T>* typed = dynamic_cast<const Node_typed<T>*>(this);
  if(!typed) return NULL;
  return &typed->value;
}

template<class T> std::shared_ptr<T> Node::getPtr() const {
  NIY
//  std::shared_ptr<T> typed = std::dynamic_pointer_cast<T>(std::shared_ptr<T>(value_ptr));
  return std::shared_ptr<T>();
//  const Node_typed<std::shared_ptr<T>>* typed = dynamic_cast<const Node_typed<std::shared_ptr<T>>*>(this);
//  if(!typed) return NULL;
//  return typed->value;
}

template<class T> Nod::Nod(const char* key, const T& x){
  n = G.newNode<T>(x);
  n->keys.append(STRING(key));
}

template<class T> Nod::Nod(const char* key, const StringA& parents, const T& x)
  : parents(parents){
  n = G.newNode<T>(x);
  n->keys.append(STRING(key));
}

template<class T> T& Graph::get(const char *key) const {
  Node *n = findNodeOfType(typeid(T), {key});
  if(!n) HALT("no node of type '" <<typeid(T).name() <<"' with key '"<< key<< "' found");
  return n->get<T>();
}

template<class T> T& Graph::get(const StringA& keys) const {
  Node *n = findNodeOfType(typeid(T), keys);
  if(!n) HALT("no node of type '" <<typeid(T).name() <<"' with keys '"<< keys<< "' found. Here is the full Graph:" <<*this);
  return n->get<T>();
}

template<class T> const T& Graph::get(const char *key, const T& defaultValue) const{
  Node *n = findNodeOfType(typeid(T), {key});
  if(!n) return defaultValue;
  return n->get<T>();
}

template<class T> mlr::Array<T*> Graph::getValuesOfType(const char* key) {
  NodeL nodes;
  if(!key) nodes = findNodesOfType(typeid(T));
  else nodes = findNodesOfType(typeid(T), {key});
  mlr::Array<T*> ret;
  for(Node *n: nodes) ret.append(n->getValue<T>());
  return ret;
}

template<class T> Node_typed<T> *Graph::newNode(const StringA& keys, const NodeL& parents, const T& x){
  return new Node_typed<T>(*this, keys, parents, x);
}

template<class T> Node_typed<T> *Graph::newNode(const StringA& keys, const NodeL& parents){
  return new Node_typed<T>(*this, keys, parents);
}

template<class T> Node_typed<T> *Graph::newNode(const T& x){
  return new Node_typed<T>(*this, x);
}

//===========================================================================

// macro for declaring types (in *.cpp files)
#define REGISTER_TYPE(Key, T) \
  RUN_ON_INIT_BEGIN(Decl_Type##_##Key) \
  registry()->newNode<std::shared_ptr<Type> >({mlr::String("Decl_Type"), mlr::String(#Key)}, NodeL(), std::make_shared<Type_typed_readable<T> >()); \
  RUN_ON_INIT_END(Decl_Type##_##Key)

#endif

/// @} //end group
