/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "graph.h"
#include "util.ipp"
#include "array.ipp"

#include <map>

#ifdef RAI_JSON
#  include <jsoncpp/json/json.h>
#endif

#define DEBUG(x) //x

rai::Graph __NoGraph;
rai::Graph& NoGraph = __NoGraph;

rai::NodeL& NoNodeL=*((rai::NodeL*)nullptr);
//Graph& NoGraph=*((Graph*)nullptr);

//===========================================================================
//
// annotations to a node while parting; can be used for highlighting and error messages
//

namespace rai {

struct ParseInfo {
  istream::pos_type beg, end;
  istream::pos_type err_beg, err_end;
  istream::pos_type keys_beg, keys_end;
  istream::pos_type parents_beg, parents_end;
  istream::pos_type value_beg, value_end;
  enum Error { good=0, unknownParent };
  void write(ostream& os) const { os <<'<' <<beg <<',' <<end <<'>'; }
};
stdOutPipe(ParseInfo)

//===========================================================================
//
// retrieving types
//

//-- query existing types
inline Node* reg_findType(const char* key) {
  NodeL types = getParameters()->getNodesOfType<std::shared_ptr<Type>>();
  for(Node* ti: types) {
    if(String(ti->get<std::shared_ptr<Type>>()->typeId().name())==key) return ti;
    if(ti->matches(key)) return ti;
  }
  return nullptr;
}

//===========================================================================
//
// read a value from a stream by looking up available registered types
//

inline Node* readTypeIntoNode(Graph& container, const char* key, std::istream& is) {
  Node* ti = reg_findType(key);
  if(ti) return ti->get<std::shared_ptr<Type>>()->readIntoNewNode(container, is);
  return nullptr;
}

//===========================================================================
//
//  Node methods
//

Node::Node(const std::type_info& _type, Graph& _container, const char* _key, const NodeL& _parents)
  : type(_type), container(_container), key(_key) {
  CHECK(&container!=&NoGraph, "This is a NGraph (nullptr) -- don't do that anymore!");
  index=container.N;
  container.NodeL::append(this);
  if(_parents.N) for(Node* p: _parents) addParent(p);
}

Node::~Node() {
  if(container.isDoubleLinked) while(children.N) children.last()->removeParent(this);
  if(numChildren) LOG(-2) <<"It is not allowed to delete nodes that still have children";
  while(parents.N) removeParent(parents.last());
  if(this==container.last()) { //great: this is very efficient to remove without breaking indexing
    container.resizeCopy(container.N-1);
  } else {
    container.removeValue(this);
    container.isIndexed=false;//  container.index();
  }
}

void Node::addParent(Node* p, bool prepend) {
  CHECK(p, "you gave me a nullptr parent");
  if(!prepend)
    parents.append(p);
  else
    parents.prepend(p);
  p->numChildren++;
  if(container.isDoubleLinked) p->children.append(this);
}

void Node::removeParent(Node* p) {
  if(p==parents.last()) parents.removeLast(); else parents.removeValue(p);
  CHECK(p->numChildren, "");
  p->numChildren--;
  if(container.isDoubleLinked) p->children.removeValue(this);
}

void Node::swapParent(uint i, Node* p) {
  CHECK(p, "you gave me a nullptr parent");
  parents(i)->numChildren--;
  if(container.isDoubleLinked) parents(i)->children.removeValue(this);
  parents(i) = p;
  parents(i)->numChildren++;
  if(container.isDoubleLinked) parents(i)->children.append(this);
}

bool Node::matches(const char* _key) {
  if(key==_key) return true;
  return false;
}

bool Node::matches(const StringA& query_keys) {
  for(const String& k:query_keys) {
    if(!matches(k)) return false;
  }
  return true;
}

void Node::write(std::ostream& os, bool yamlMode) const {
  if(!container.isIndexed) container.index();

  //-- write keys
#if 0
  if(key.N) {
    if(yamlMode) {
      if(key.N!=1) os <<'"';
      key.write(os, " ", "", "\0\0");
//      keys.write(os, " ", "", "\"\"");
      if(key.N!=1) os <<'"';
    } else
      key.write(os, " ", "", "\0\0");
  }
#else
  if(key.N) key.write(os);
#endif

  //-- write parents
  if(parents.N) {
    //    if(keys.N) os <<' ';
    os <<'(';
    for(Node* it: parents) {
      if(it!=parents.first()) os <<' ';
      if(it->key.N) {
        os <<it->key;
      } else { //relative numerical reference
        os <<(int)it->index - (int)index;
      }
    }
    os <<')';
  }

  //-- boolean special
  if(isOfType<bool>()) {
    bool x = *getValue<bool>();
    if(yamlMode) { if(x) os <<": True"; else os <<": False"; }
    else { if(!x) os <<'!'; }
    return;
  }

  //-- colon separator
  if(key.N || parents.N) {
    if(yamlMode) os <<": ";
    else os <<':';
  }

  //-- write value
  if(isGraph()) {
    graph().write(os, ", ", "{  }", yamlMode);
  } else if(isOfType<NodeL>()) {
    os <<"(";
    for(Node* it: (*getValue<NodeL>())) os <<' ' <<it->key;
    os <<" )";
  } else if(isOfType<String>()) {
    if(yamlMode) {
      os <<'"' <<*getValue<String>() <<'"';
    } else {
      const String& str = *getValue<String>();
      bool onlyLetters=true;
      for(uint i=0; i<str.N; i++) {
        char c=str(i);
        if(!((c>='a'&& c<='z') || (c>='A'&& c<='Z'))) {
          onlyLetters=false;
          break;
        }
      }
      if(onlyLetters) os <<str;
      else os <<'"' <<str <<'"';
    }
  } else if(isOfType<FileToken>()) {
    os <<'\'' <<getValue<FileToken>()->absolutePathName() <<'\'';
//    os <<'\'' <<getValue<FileToken>()->name <<'\'';
  } else if(isOfType<arr>()) {
    getValue<arr>()->write(os, ", ", nullptr, "[]");
  } else if(isOfType<intA>()) {
    getValue<intA>()->write(os, ", ", nullptr, "[]");
  } else if(isOfType<intAA>()) {
    getValue<intAA>()->write(os, ", ", nullptr, "[]");
  } else if(isOfType<uintA>()) {
    getValue<uintA>()->write(os, ", ", nullptr, "[]");
  } else if(isOfType<StringA>()) {
    os <<"[";
    for(const String& s:get<StringA>()) os <<'\"' <<s <<"\", ";
    os <<']';
  } else if(isOfType<double>()) {
    os <<*getValue<double>();
  } else if(isOfType<int>()) {
    os <<*getValue<int>();
  } else if(isOfType<uint>()) {
    os <<*getValue<uint>();
  } else if(isOfType<Type*>()) {
    get<Type*>()->write(os);
  } else {
    writeValue(os);
  }
}

NodeInitializer::NodeInitializer(const char* key) {
  n = G.newNode<bool>(key, {}, true);
}

NodeInitializer::NodeInitializer(const char* key, const char* stringValue) {
  n = G.newNode<String>(key, {}, STRING(stringValue));
}

//===========================================================================
//
//  Graph methods
//

Graph::Graph() : isNodeOfGraph(nullptr), pi(nullptr), ri(nullptr) {
}

Graph::Graph(const char* filename): Graph() {
  FileToken file(filename, true);
  read(file);
  file.cd_start();
}

Graph::Graph(istream& is) : Graph() {
  read(is);
}

Graph::Graph(const std::map<std::string, std::string>& dict) : Graph() {
  appendDict(dict);
}

Graph::Graph(std::initializer_list<NodeInitializer> list) : Graph() {
  for(const NodeInitializer& ni:list) newNode(ni);
}

//Graph::Graph(std::initializer_list<const char*> list) : Graph(){
//  for(const char* s:list){
//    String str(s);
//    Graph &g=newSubgraph();
//    g.read(str);
//  }
//}

Graph::Graph(const Graph& G) : Graph() {
  *this = G;
}

Graph::~Graph() {
  clear();
}

bool Graph::operator!() const {
  return this==&__NoGraph;
}

void Graph::clear() {
  if(ri) { delete ri; ri=nullptr; }
  if(pi) { delete pi; pi=nullptr; }
  DEBUG(checkConsistency();)
  if(!isNodeOfGraph) { //this is not a subgraph; save to delete connections in batch -> faster
    NodeL all = getAllNodesRecursively();
    for(Node* n:all) {
      n->parents.clear();
      n->numChildren=0;
      n->children.clear();
      n->key.clear();
    }
    DEBUG(checkConsistency();)
  }
  //delete all subgraphs first to remove potential children
  for(Node* n:*this) if(n->isGraph()) n->graph().clear();
  while(N) {
    Node** n = NodeL::p+N-1; //last
    if(!isDoubleLinked) while((*n)->numChildren) { n--; CHECK_GE(n, p, "can't find a node without children"); }
    delete *n;
  }
  isIndexed=true;
}

Graph& Graph::newNode(const NodeInitializer& ni) {
  Node* clone = ni.n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
  for(const String& s:ni.parents) {
    Node* p = findNode(s, true, false);
    CHECK(p, "parent " <<p <<" of " <<*clone <<" does not exist!");
    clone->addParent(p);
  }
  return *this;
}

Graph& Graph::newSubgraph(const char* key, const NodeL& parents, const Graph& x) {
  Node_typed<Graph>* n = newNode<Graph>(key, parents, Graph());
  DEBUG(CHECK(n->value.isNodeOfGraph && &n->value.isNodeOfGraph->container==this, ""))
  if(!!x) n->value.copy(x);
  n->value.isDoubleLinked = isDoubleLinked;
  return n->value;
}

Node_typed<int>* Graph::newNode(const uintA& parentIdxs) {
  NodeL parents(parentIdxs.N);
  for(uint i=0; i<parentIdxs.N; i++) parents(i) = NodeL::elem(parentIdxs(i));
  return newNode<int>(STRING(NodeL::N), parents, 0);
}

void Graph::appendDict(const std::map<std::string, std::string>& dict) {
  for(const std::pair<std::string, std::string>& p:dict) {
    StringA tags;
    Node* n = readNode(STRING(':'<<p.second), tags, p.first.c_str(), false, false);
    if(!n) RAI_MSG("failed to read dict entry <" <<p.first <<',' <<p.second <<'>');
  }
}

Node* Graph::findNode(const char* key, bool recurseUp, bool recurseDown) const {
  for(Node* n: (*this)) if(n->matches(key)) return n;
  Node* ret=nullptr;
  if(recurseUp && isNodeOfGraph) ret = isNodeOfGraph->container.findNode(key, true, false);
  if(ret) return ret;
  if(recurseDown) for(Node* n: (*this)) if(n->isGraph()) {
        ret = n->graph().findNode(key, false, true);
        if(ret) return ret;
      }
  return ret;
}

Node* Graph::findNodeOfType(const std::type_info& type, const char* key, bool recurseUp, bool recurseDown) const {
  for(Node* n: (*this)) if(n->type==type && (!key || n->matches(key))) return n;
  Node* ret=nullptr;
  if(recurseUp && isNodeOfGraph) ret = isNodeOfGraph->container.findNodeOfType(type, key, true, false);
  if(ret) return ret;
  if(recurseDown) for(Node* n: (*this)) if(n->isGraph()) {
        ret = n->graph().findNodeOfType(type, key, false, true);
        if(ret) return ret;
      }
  return ret;
}

NodeL Graph::findNodes(const char* key, bool recurseUp, bool recurseDown) const {
  NodeL ret;
  for(Node* n: (*this)) if(n->matches(key)) ret.append(n);
  if(recurseUp && isNodeOfGraph) ret.append(isNodeOfGraph->container.findNodes(key, true, false));
  if(recurseDown) for(Node* n: (*this)) if(n->isGraph()) ret.append(n->graph().findNodes(key, false, true));
  return ret;
}

NodeL Graph::findNodesOfType(const std::type_info& type, const char* key, bool recurseUp, bool recurseDown) const {
  NodeL ret;
  for(Node* n: (*this)) if(n->type==type && (!key || n->matches(key))) ret.append(n);
  if(recurseUp && isNodeOfGraph) ret.append(isNodeOfGraph->container.findNodesOfType(type, key, true, false));
  if(recurseDown) for(Node* n: (*this)) if(n->isGraph()) ret.append(n->graph().findNodesOfType(type, key, false, true));
  return ret;
}

NodeL Graph::findGraphNodesWithTag(const char* tag) const {
  NodeL ret;
  for(Node* n: (*this)) if(n->isGraph() && n->graph().findNode(tag)) ret.append(n);
  return ret;
}

//Node* Graph::getNode(const char *key) const {
//  for(Node *n: (*this)) if(n->matches(key)) return n;
//  if(isNodeOfGraph) return isNodeOfGraph->container.getNode(key);
//  return nullptr;
//}

//Node* Graph::getNode(const StringA &keys) const {
//  for(Node *n: (*this)) if(n->matches(keys)) return n;
//  if(isNodeOfGraph) return isNodeOfGraph->container.getNode(keys);
//  return nullptr;
//}

//NodeL Graph::getNodes(const StringA &keys) const {
//  NodeL ret;
//  for(Node *n: (*this)) if(n->matches(keys)) ret.append(n);
//  return ret;

//}

//NodeL Graph::getNodes(const char* key) const {
//  NodeL ret;
//  for(Node *n: (*this)) if(n->matches(key)) ret.append(n);
//  return ret;
//}

Node* Graph::getEdge(Node* p1, Node* p2) const {
  if(p1->children.N < p2->children.N) {
    for(Node* i:p1->children) {
      if(p2->children.findValue(i)!=-1) return i;
    }
  } else {
    for(Node* i:p2->children) {
      if(p1->children.findValue(i)!=-1) return i;
    }
  }
  return nullptr;
}

Node* Graph::getEdge(const NodeL& parents) const {
  CHECK(parents.N>0, "");
  //grap 'sparsest' parent:
  uint minSize = this->N;
  Node* sparsestParent = nullptr;
  for(Node* p:parents) if(p->children.N<minSize) { sparsestParent=p; minSize=p->children.N; }
  if(!sparsestParent) {
    for(Node* e:*this) if(e->parents==parents) return e;
  } else {
    for(Node* e:sparsestParent->children) if(&e->container==this) {
        if(e->parents==parents) return e;
      }
  }
  return nullptr;
}

NodeL Graph::getNodesOfDegree(uint deg) {
  NodeL ret;
  for(Node* n: (*this)) if(n->parents.N==deg) ret.append(n);
  return ret;
}

NodeL Graph::getAllNodesRecursively() const {
  NodeL ret = *this;
  NodeL below;
  for(Node* n:ret) if(n->isGraph()) below.append(n->graph().getAllNodesRecursively());
  ret.append(below);
  return ret;
}

Node* Graph::edit(Node* ed) {
  NodeL KVG = findNodesOfType(ed->type, ed->key);
  //CHECK_LE(KVG.N, 1, "can't edit into multiple nodes yet");
  if(!KVG.N) { //nothing to merge, append
    if(&ed->container!=this) {
      if(!isIndexed) index();
      if(!ed->container.isIndexed) ed->container.index();
      Node* it = ed->newClone(*this);
      for(uint i=0; i<it->parents.N; i++) {
        it->swapParent(i, elem(it->parents(i)->index));
      }
    }
    return ed;
  }

  uint edited=0;
  for(Node* n : KVG) if(n!=ed) {
      CHECK(ed->type == n->type, "can't edit/merge nodes of different types!");
      if(ed->parents.N) { //replace parents
        while(n->parents.N) n->removeParent(n->parents.last());
        for(Node* p:ed->parents) n->addParent(p);
      }
      if(n->isGraph()) { //merge the KVGs
        n->graph().edit(ed->graph());
      } else { //overwrite the value
        n->copyValue(ed);
      }
      edited++;
    }
  if(!edited) {
    RAI_MSG("no nodes edited!");
  }
  if(&ed->container==this) { delete ed; ed=nullptr; }
  return nullptr;
}

void Graph::collapse(Node* a, Node* b) {
  NodeL ab= {a, b}, ba= {b, a};
//  cout <<"collapsing " <<a->keys.first() <<' ' <<b->keys.first() <<endl;
//  cout <<"collapsing " <<*a <<listString(a->children) <<" and " <<*b <<listString(b->children) <<endl;
//  a->keys.first() <<'_' <<b->keys.first();
  for(Node* ch:a->children) if(ch->parents==ab || ch->parents==ba) delete ch;
  NodeL b_parentOf = b->children;
  for(Node* ch:b_parentOf) {
    for(Node*& p:ch->parents) if(p==b) {
        p=a;
        b->children.removeValue(ch);
        b->numChildren--;
        a->children.prepend(ch);
        a->numChildren++;
      }
  }
//  cout <<"... becomes " <<*a <<listString(a->children) <<" and " <<*b <<listString(b->children) <<endl;
//  checkConsistency();
  delete b;
}

void Graph::copy(const Graph& G, bool appendInsteadOfClear, bool enforceCopySubgraphToNonsubgraph) {
  DEBUG(G.checkConsistency());
  if(!G.isIndexed) HALT("can't copy non-indexed graph");

  CHECK(this!=&G, "Graph self copy -- never do this");

  if(!enforceCopySubgraphToNonsubgraph) {
    if(G.isNodeOfGraph && !this->isNodeOfGraph) {
      HALT("Typically you should not copy a subgraph into a non-subgraph (or call the copy operator with a subgraph).\
           Use 'newSubgraph' instead\
           If you still want to do it you need to ensure that all node parents are declared, and then enforce it by setting 'enforceCopySubgraphToNonsubgraph'");
    }
  } else {
    if(this->isNodeOfGraph) {
      HALT("You set 'enforceCopySubgraphToNonsubgraph', but this is not a Nonsubgraph");
    }
  }

  //-- first delete existing nodes
  if(!appendInsteadOfClear) clear();
  uint indexOffset=N;
  NodeL newNodes;

  //-- if either is a subgraph, ensure they're a subgraph of the same -- over restrictive!!
  //  if(isNodeOfGraph || G.isNodeOfGraph){
  //    CHECK(&isNodeOfGraph->container==&G.isNodeOfGraph->container,"is already subgraph of another container!");
  //  }

  //-- first, just clone nodes with their values -- 'parents' still point to the origin nodes
  for(Node* n:G) {
    Node* newn=nullptr;
    if(n->isGraph()) {
      // why we can't copy the subgraph yet:
      // copying the subgraph would require to fully rewire the subgraph (code below)
      // but if the subgraph refers to parents of this graph that are not create yet, requiring will fail
      // therefore we just insert an empty graph here; we then copy the subgraph once all nodes are created
      newn = this->newSubgraph(n->key, n->parents).isNodeOfGraph;
    } else {
      newn = n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
    }
    newNodes.append(newn);
  }

  //-- the new nodes are not parent of anybody yet
#ifndef RAI_NOCHECK
  for(Node* n:newNodes) CHECK(n->numChildren==0 && n->children.N==0, "");
#endif

  //-- now copy subgraphs
  for(Node* n:newNodes) if(n->isGraph()) {
      n->graph().copy(G.elem(n->index-indexOffset)->graph()); //you can only call the operator= AFTER assigning isNodeOfGraph
    }

  //-- now rewire parental links
  for(Node* n:newNodes) {
    for(uint i=0; i<n->parents.N; i++) {
      Node* p=n->parents(i); //the parent in the origin graph
      Node* newp=nullptr;
      if(isChildOfGraph(p->container)) continue;
      if(&p->container==&G) { //parent is directly in G, no need for complicated search
        newp = newNodes.elem(p->index);  //the true parent in the new graph
      } else {
        const Graph* newg=this, *oldg=&G;
        while(&p->container!=oldg) { //find the container while iterating backward also in the newG
          CHECK(oldg->isNodeOfGraph, "");
          CHECK(newg->isNodeOfGraph, "");
          newg = &newg->isNodeOfGraph->container;
          oldg = &oldg->isNodeOfGraph->container;
        }
        CHECK_EQ(newg->N, oldg->N, "different size!!\n" <<*newg <<"**\n" <<*oldg);
        CHECK_EQ(p, oldg->elem(p->index), ""); //we found the parent in oldg
        newp = newg->elem(p->index);     //the true parent in the new graph
      }
      n->swapParent(i, newp);
    }
  }

  DEBUG(this->checkConsistency());
  DEBUG(G.checkConsistency());
}

void Graph::read(std::istream& is, bool parseInfo) {
  uint Nbefore = N;
  if(parseInfo) getParseInfo(nullptr).beg=is.tellg();
  String namePrefix;
  StringA tags;
  for(;;) {
    DEBUG(checkConsistency());
    char c=peerNextChar(is, " \n\r\t,");
    if(!is.good() || c=='}') { is.clear(); break; }
    Node* n = readNode(is, tags, NULL, false, parseInfo);
    if(!n) break;
    if(n->key=="Quit") {
      delete n; n=nullptr;
    }
    if(!n) break;
    if(n->key=="Include") {
      uint Nbefore = N;
      read(n->get<FileToken>().getIs(true), parseInfo);
      if(namePrefix.N) { //prepend a naming prefix to all nodes just read
        for(uint i=Nbefore; i<N; i++) elem(i)->key.prepend(namePrefix);
        namePrefix.clear();
      }
      n->get<FileToken>().cd_start();
      delete n; n=nullptr;
    } else if(n->key=="Prefix") {
      if(n->isOfType<String>()) {
        namePrefix = n->get<String>();
      } else if(n->isOfType<bool>() && !n->get<bool>()) {
        namePrefix.clear();
      } else LOG(-1) <<*n <<" is not a proper name prefix";
      delete n; n=nullptr;
    } else if(n->key=="ChDir") {
      n->get<FileToken>().cd_file();
    } else if(tags.N && tags(0)=="Delete") {
//      n->key.remove(0);
      NodeL dels = getNodes(n->key);
      for(Node* d: dels) { delete d; d=nullptr; }
    }
  }
  if(parseInfo) getParseInfo(nullptr).end=is.tellg();

  DEBUG(checkConsistency());

  //-- merge all Mege keys
  NodeL edits;// = getNodesWithTag("%Edit");
  for(uint i=Nbefore; i<N; i++) {
    Node* n=elem(i);
    if(n->isGraph() && n->graph().findNode("%Edit")) edits.append(n);
  }
  for(Node* ed:edits) {
//    CHECK_EQ(ed->key.first(), "Edit", "an edit node needs Edit as first key");
    ed->graph().delNode(ed->graph().findNode("%Edit"));
//    ed->key.remove(0);
    edit(ed);
  }

  DEBUG(checkConsistency();)

  //-- delete all ChDir nodes in reverse order
  for(uint i=N; i--;) {
    Node* n=elem(i);
    if(n->key=="ChDir") {
      n->get<FileToken>().cd_start();
      delete n; n=nullptr;
    }
  }

  index();
}

void writeFromStream(std::ostream& os, std::istream& is, istream::pos_type beg, istream::pos_type end) {
  istream::pos_type here=is.tellg();
  is.seekg(beg);
  if(beg>end){
    char c;
    for(int i=int(beg - end); i--;) {
      is.get(c);
      os <<c;
    }
  }
  is.seekg(here);
}

#define PARSERR(x, pinfo) { \
    cerr <<"[[error in parsing Graph file (line=" <<lineCount <<"): " <<x <<":\n  \""; \
    writeFromStream(cerr, is, pinfo.beg, is.tellg()); \
    cerr <<"<<<\"  ]]" <<endl; \
    is.clear(); }

//  if(node) cerr <<"  (node='" <<*node <<"')" <<endl;

void readNodeParents(Graph& G, std::istream& is, NodeL& parents, ParseInfo& pinfo) {
  String str;
  pinfo.parents_beg=is.tellg();
  for(uint j=0;; j++) {
    if(!str.read(is, " \t\n\r,", " \t\n\r,)", false)) break;
    Node* e = G.findNode(str, true, false); //important: recurse up
    if(e) { //sucessfully found
      parents.append(e);
      pinfo.parents_end=is.tellg();
    } else { //this element is not known!!
      int rel=0;
      str >>rel;
      if(rel<0 && (int)G.N+rel>=0) { //check if this is a negative integer
        e=G.elem(G.N+rel);
        parents.append(e);
        pinfo.parents_end=is.tellg();
      } else {
        PARSERR("unknown " <<j <<". parent '" <<str <<"'", pinfo);
        skip(is, nullptr, ")", false);
      }
    }
  }
  parse(is, ")");
}

Node* Graph::readNode(std::istream& is, StringA& tags, const char* predeterminedKey, bool verbose, bool parseInfo) {
  String str;

  ParseInfo pinfo;
  pinfo.beg=is.tellg();

  if(verbose) { cout <<"\nNODE (line="<<lineCount <<")"; }

  //-- read keys
  tags.clear();
  skip(is, " \t\n\r");
  pinfo.keys_beg=is.tellg();
  for(;;) {
    if(!str.read(is, " \t", " \t\n\r,;([{}=:!\'", false)) break;
    if(str(0)=='"' && str(-1)=='"') str = str.getSubString(1, -2);
    tags.append(str);
    pinfo.keys_end=is.tellg();
  }
  DEBUG(checkConsistency());

  if(verbose) { cout <<" tags:" <<tags <<flush; }

  String key;
  if(!predeterminedKey) {
    if(tags.N) key = tags.last();
  } else {
    key = predeterminedKey;
  }

  //-- read parents
  NodeL parents;
  char c=getNextChar(is, " \t"); //don't skip new lines
  if(c=='(') {
    readNodeParents(*this, is, parents, pinfo);
    c=getNextChar(is, " \t");
  }
  DEBUG(checkConsistency());

  if(verbose) { cout <<" parents:"; if(!parents.N) cout <<"none"; else listWrite(parents, cout, " ", "()"); cout <<flush; }

  //-- read value
  Node* node=nullptr;
  pinfo.value_beg=(long int)is.tellg()-1;
  if(c=='=' || c==':' || c=='{' || c=='[' || c=='<' || c=='!' || c=='\'') {
    if(c=='=' || c==':') c=getNextChar(is, " \t");
    if((c>='a' && c<='z') || (c>='A' && c<='Z') || c=='_') { //String or boolean
      is.putback(c);
      str.read(is, "", " \n\r\t,;}", false);
      if(str=="true") node = newNode<bool>(key, parents, true);
      else if(str=="false") node = newNode<bool>(key, parents, false);
      else node = newNode<String>(key, parents, str);
    } else if(rai::contains("-.0123456789", c)) {  //single double
      is.putback(c);
      double d;
      try { is >>d; } catch(...) PARSERR("can't parse the double number", pinfo);
      node = newNode<double>(key, parents, d);
    } else switch(c) {
        case '!': { //boolean false
          node = newNode<bool>(key, parents, false);
        } break;
        case '\'': { //FileToken
          str.read(is, "", "\'", true);
          try {
            node = newNode<FileToken>(key, parents, FileToken(str, false));
//          node->get<FileToken>().getIs();  //creates the ifstream and might throw an error
          } catch(...) {
            delete node; node=nullptr;
            PARSERR("file " <<str <<" does not exist -> converting to string!", pinfo);
            node = newNode<String>(key, parents, str);
          }
        } break;
        case '\"': { //String
          str.read(is, "", "\"", true);
          node = newNode<String>(key, parents, str);
        } break;
        case '[': { //arr or StringA
          char c2=getNextChar(is, " \t");
          if(c2=='"') { //StringA
            is.putback(c2);
            is.putback(c);
            StringA strings;
            String::readSkipSymbols=",\"";
            String::readStopSymbols="\"";
            String::readEatStopSymbol = 1;
            is >>strings;
            String::readSkipSymbols = " \t";
            String::readStopSymbols = "\n\r";
            String::readEatStopSymbol = 1;
            node = newNode<StringA>(key, parents, strings);
          } else if(c2=='[') { //arrA
            is.putback(c2);
            is.putback(c);
            arrA reals;
            is >>reals;
            node = newNode<arrA>(key, parents, reals);
          } else if((c2>='a' && c2<='z') || (c2>='A' && c2<='Z')) { //StringA}
            is.putback(c2);
            is.putback(c);
            StringA strings;
            String::readStopSymbols=" \n\t]";
            String::readEatStopSymbol = 0;
            is >>strings;
            String::readStopSymbols = "\n\r";
            String::readEatStopSymbol = 1;
            node = newNode<StringA>(key, parents, strings);
          } else {
            is.putback(c2);
            is.putback(c);
            arr reals;
            is >>reals;
            node = newNode<arr>(key, parents, reals);
          }
        } break;
        case '<': { //any type parser
#if 1
          str.read(is, "", ">", true);
          node = newNode<String>(key, parents, str);
#else
          str.read(is, " \t", " \t\n\r()`-=~!@#$%^&*()+[]{};'\\:|,./<>?", false);
          //      str.read(is, " \t", " \t\n\r()`1234567890-=~!@#$%^&*()_+[]{};'\\:|,./<>?", false);
          //        node = readTypeIntoNode(*this, str, is);
          if(!node) {
            is.clear();
            String substr;
            substr.read(is, "", ">", false);
            //          PARSERR("could not parse value of type '" <<str <<"' -- no such type has been registered; converting this to string: '"<<substr<<"'", pinfo);
            str = STRING('<' <<str <<' ' <<substr <<'>');
            node = newNode<String>(keys, parents, str);
          } else {
            node->keys = keys;
            node->parents = parents;
          }
          parse(is, ">");
#endif
        } break;
        case '(': { // set of parent nodes
          NodeL par;
          readNodeParents(*this, is, par, pinfo);
          node = newNode<NodeL>(key, parents, par);
        } break;
        case '{': { // sub graph
          Graph& subgraph = this->newSubgraph(key, parents);
          subgraph.read(is);
          parse(is, "}");
          node = subgraph.isNodeOfGraph;
          if(tags.N>1) {
            for(uint i=0; i<tags.N-1; i++) subgraph.newNode<bool>(STRING('%' <<tags.elem(i)));
          }
        } break;
        default: { //error
          is.putback(c);
          PARSERR("unknown value indicator '" <<c <<"'", pinfo);
          return nullptr;
        }
      }
  } else { //no ':' or '{' -> boolean
    is.putback(c);
    node = newNode<bool>(key, parents, true);
  }
  if(node) pinfo.value_end=is.tellg();
  pinfo.end=is.tellg();
  DEBUG(checkConsistency();)

  if(parseInfo && node) node->container.getParseInfo(node) = pinfo;

  if(verbose) {
    if(node) { cout <<" value:"; node->writeValue(cout); cout <<" FULL:"; node->write(cout); cout <<endl; }
    else { cout <<"FAILED" <<endl; }
  }

  if(!node) {
    cerr <<"FAILED reading node with keys ";
    tags.write(cerr, " ", nullptr, "()");
    cerr <<" and parents ";
    listWrite(parents, cerr, " ", "()");
    cerr <<endl;
  }

  if(tags.N>1) {
    if(node->isOfType<bool>() && tags.N==2 && tags(0)=="Delete") {
      node->get<bool>() = false;
    } else if(!node->isGraph()) {
      LOG(-1) <<"you specified tags " <<tags <<" for node '" <<*node <<"', which is of non-graph type -- ignored";
    }
  }

  //eat the next , or ;
  c=getNextChar(is, " \n\r\t");
  if(c==',' || c==';') {} else is.putback(c);

  return node;
}

#ifdef RAI_JSON
void addJasonValues(Graph& G, const char* key, Json::Value& value);

void Json2Graph(Graph& G, Json::Value& value) {
  CHECK_EQ(value.type(), Json::objectValue, "needs an object type");
  for(auto& n:value.getMemberNames()) {
    addJasonValues(G, n.c_str(), value[n]);
  }
}

void addJasonValues(Graph& G, const char* key, Json::Value& value) {
  if(value.type()==Json::arrayValue) {
    CHECK(value.size()>0, "");
    if(value[0].isConvertibleTo(Json::realValue)) { //.type()==Json::realValue //convert int to double
      arr x(value.size());
      for(uint i=0; i<x.d0; i++) x(i) = value[i].asDouble();
      new Node_typed<arr>(G, {key}, {}, x);
    } else if(value[0].type()==Json::intValue) {
      intA x(value.size());
      for(uint i=0; i<x.N; i++) x(i) = value[i].asInt();
      new Node_typed<intA>(G, {key}, {}, x);
    } else if(value[0].type()==Json::stringValue) {
      StringA x(value.size());
      for(uint i=0; i<x.N; i++) x(i) = value[i].asString().c_str();
      new Node_typed<StringA>(G, {key}, {}, x);
    } else if(value[0].type()==Json::arrayValue) {
      if(value[0][0].isConvertibleTo(Json::realValue)) { //.type()==Json::realValue //convert int to double
        arr x(value.size(), value[0].size());
        for(uint i=0; i<x.d0; i++) for(uint j=0; j<x.d1; j++) x(i, j) = value[i][j].asDouble();
        new Node_typed<arr>(G, {key}, {}, x);
      } else if(value[0][0].type()==Json::intValue) {
        intA x(value.size(), value[0].size());
        for(uint i=0; i<x.d0; i++) for(uint j=0; j<x.d1; j++) x(i, j) = value[i][j].asInt();
        new Node_typed<intA>(G, {key}, {}, x);
      } else if(value[0][0].type()==Json::arrayValue) {
        if(value[0][0][0].isConvertibleTo(Json::realValue)) { //.type()==Json::realValue //convert int to double
          arr x(value.size(), value[0].size(), value[0][0].size());
          for(uint i=0; i<x.d0; i++) for(uint j=0; j<x.d1; j++) for(uint k=0; k<x.d2; k++)
                x(i, j, k) = value[i][j][k].asDouble();
          new Node_typed<arr>(G, {key}, {}, x);
        } else if(value[0][0][0].type()==Json::intValue) {
          intA x(value.size(), value[0].size(), value[0][0].size());
          for(uint i=0; i<x.d0; i++) for(uint j=0; j<x.d1; j++) for(uint k=0; k<x.d2; k++)
                x(i, j, k) = value[i][j][k].asInt();
          new Node_typed<intA>(G, {key}, {}, x);
        } else {
          cout <<value[0][0][0].type();
          NIY;
        }
      } else {
        cout <<value[0][0].type();
        NIY;
      }
    } else {
      cout <<value[0].type();
      NIY
    }
  } else {
    switch(value.type()) {
      case Json::nullValue: NIY; break;
      case Json::intValue: //new Node_typed<int>(G, {key}, {}, value.asInt()); break; //convert int to double
      case Json::uintValue: //new Node_typed<uint>(G, {key}, {}, value.asUInt()); break; //convert int to double
      case Json::realValue: new Node_typed<double>(G, {key}, {}, value.asDouble()); break;
      case Json::stringValue: new Node_typed<String>(G, {key}, {}, value.asString().c_str()); break;
      case Json::booleanValue: new Node_typed<bool>(G, {key}, {}, value.asBool()); break;
      case Json::arrayValue: HALT("covered above"); break;
      case Json::objectValue:  Json2Graph(G.newSubgraph({key}, {}), value);  break;
    }
  }
}

void Graph::readJson(std::istream& is) {
  Json::Value root;   // 'root' will contain the root value after parsing.
  is >>root;
  Json2Graph(*this, root);
}
#else
void Graph::readJson(std::istream& is) {
  NICO
}
#endif

#undef PARSERR

void Graph::write(std::ostream& os, const char* ELEMSEP, const char* BRACKETS, bool yamlMode) const {
  uint BRACKETSlength=0;
  if(BRACKETS) BRACKETSlength=strlen(BRACKETS);
  for(uint b=0; b<BRACKETSlength/2; b++) os <<BRACKETS[b];
  for(uint i=0; i<N; i++) { if(i) os <<ELEMSEP;  if(elem(i)) elem(i)->write(os, yamlMode); else os <<"<nullptr>"; }
  for(uint b=BRACKETSlength/2; b<BRACKETSlength; b++) os <<BRACKETS[b];
  os <<std::flush;
}

void Graph::writeParseInfo(std::ostream& os) {
  os <<"GRAPH " <<getParseInfo(nullptr) <<endl;
  for(Node* n:*this)
    os <<"NODE '" <<*n <<"' " <<getParseInfo(n) <<endl;
}

void Graph::displayDot(Node* highlight) {
  if(highlight) {
    CHECK(&highlight->container==this, "");
    writeDot(FILE("z.dot"), false, false, 0, highlight->index, true);
  } else {
    writeDot(FILE("z.dot"), false, false, 0, -1, true);
  }
  system("dot -Tpdf z.dot > z.pdf");
  system("evince z.pdf &");
}

void Graph::writeHtml(std::ostream& os, std::istream& is) {
  char c;
  long int g=getParseInfo(nullptr).beg;
  is.seekg(g);
#define GO { is.get(c); if(c=='\n') os <<"<br>" <<endl; else os <<c; g++; }
  for(Node* n:list()) {
    ParseInfo& pinfo=getParseInfo(n);
    while(g<pinfo.keys_beg) GO
      os <<"<font color=\"0000ff\">";
    while(g<pinfo.keys_end) GO
      os <<"</font>";
    while(g<pinfo.parents_beg)GO
      os <<"<font color=\"00ff00\">";
    while(g<pinfo.parents_end)GO
      os <<"</font>";
    while(g<pinfo.value_beg)GO
      os <<"<font color=\"ff0000\">";
    while(g<pinfo.value_end)GO
      os <<"</font>";
  }
  while(g<getParseInfo(nullptr).end)GO
#undef GO
  }

void Graph::writeDot(std::ostream& os, bool withoutHeader, bool defaultEdges, int nodesOrEdges, int focusIndex, bool subGraphsAsNodes) {
  if(!withoutHeader) {
    os <<"digraph G{" <<endl;
    os <<"graph [ rankdir=\"LR\", ranksep=0.05";
    if(hasRenderingInfo(nullptr)) os <<' ' <<getRenderingInfo(nullptr).dotstyle;
    os << " ];" <<endl;
    os <<"node [ fontsize=9, width=.3, height=.3 ];" <<endl;
    os <<"edge [ arrowtail=dot, arrowsize=.5, fontsize=6 ];" <<endl;
    index(true);
  } else {
    if(!isIndexed) index();
  }
  for(Node* n: list()) {
    if(hasRenderingInfo(n) && getRenderingInfo(n).skip) continue;
    String label;
    if(n->key.N) {
#if 0
      bool newline=false;
      for(String& k:n->key) {
        if(newline) label <<"\\n";
        label <<k;
        newline=true;
      }
#else
      label <<n->key;
#endif
    } else {
//      if(n->parents.N) {
//        label <<"(" <<n->parents(0)->keys.last();
//        for(uint i=1; i<n->parents.N; i++) label <<' ' <<n->parents(i)->keys.last();
//        label <<")";
//      }
    }
    if(label.N) label <<"\\n";
    n->writeValue(label);

    String shape;
    if(n->key.contains("box")) shape <<", shape=box"; else shape <<", shape=ellipse";
    if(focusIndex==(int)n->index) shape <<", color=red";
    if(hasRenderingInfo(n)) shape <<' ' <<getRenderingInfo(n).dotstyle;

    if(defaultEdges && n->parents.N==2) { //an edge
      os <<n->parents(0)->index <<" -> " <<n->parents(1)->index <<" [ label=\"" <<label <<"\" ];" <<endl;
    } else {
      if(n->isGraph()) {
        bool graphNodesInside=false;
        Graph& nG = n->graph();
        for(Node* c:nG) if(c->parents.N || c->isGraph()) graphNodesInside=true;
        if(!subGraphsAsNodes && graphNodesInside) {
          os <<"subgraph cluster_" <<n->index <<" { " /*<<" rank=same"*/ <<endl;
          os <<n->index <<" [ label=\"" <<label <<"\" shape=box ];" <<endl;
          n->graph().writeDot(os, true, defaultEdges, +1);
          os <<"}" <<endl;
          n->graph().writeDot(os, true, defaultEdges, -1);
        } else {
          label <<'\n' <<n->graph();
          for(uint i=0; i<label.N; i++) if(label(i)=='"') label(i)='\'';
          os <<n->index <<" [ label=\"" <<label <<"\" shape=box ];" <<endl;
        }
      } else { //normal node
        if(nodesOrEdges>=0) {
          os <<n->index <<" [ label=\"" <<label <<'"' <<shape <<" ];" <<endl;
        }
      }
      if(nodesOrEdges<=0) {
        uint pa_COUNT=0;
        for(Node* pa: n->parents) {
          if(hasRenderingInfo(pa) && getRenderingInfo(pa).skip) continue;
          //              if(pa->index<n->index)
          os <<pa->index <<" -> " <<n->index <<" [ ";
          //              else
          //                  os <<n->index <<" -> " <<pa->index <<" [ ";
          os <<"label=" <<pa_COUNT++;
          os <<" ];" <<endl;
        }
      }
    }
  }
  if(!withoutHeader) {
    os <<"}" <<endl;
    index(false);
  }
}

void Graph::sortByDotOrder() {
  uintA perm;
  perm.setStraightPerm(N);
  uint it_COUNT=0;
  for(Node* it: list()) {
    if(it->isGraph()) {
      double* order = it->graph().find<double>("dot_order");
      if(!order) { RAI_MSG("doesn't have dot_order attribute"); return; }
      perm(it_COUNT++) = (uint)*order;
    }
  }
  permuteInv(perm);
  it_COUNT=0;
  for(Node *it: list()) it->index=it_COUNT++;
}

ParseInfo& Graph::getParseInfo(Node* n) {
  if(!pi) pi=new ArrayG<ParseInfo>(*this);
  return pi->operator()(n);
  //  if(pi.N!=N+1){
  //    listResizeCopy(pi, N+1);
  //    pi(0)->node=nullptr;
  //    for(uint i=1;i<pi.N;i++) pi(i)->node=elem(i-1);
  //  }
  //  if(!n) return *pi(0);
  //  return *pi(n->index+1);
}

RenderingInfo& Graph::getRenderingInfo(Node* n) {
  CHECK(!n || &n->container==this, "");
#if 1
  if(!ri) ri=new ArrayG<RenderingInfo>(*this);
  return ri->operator()(n);
#else
  if(ri.N!=N+1) {
    ri.resizeCopy(N+1); //listResizeCopy(ri, N+1);
    //    ri.elem(0)->node=nullptr;
    //    for(uint i=1;i<ri.N;i++) ri.elem(i)->node=elem(i-1);
  }
  if(!n) return ri.elem(0);
  return ri.elem(n->index+1);
#endif
}

const Graph* Graph::getRootGraph() const {
  const Graph* g=this;
  for(;;) {
    const Node* n=g->isNodeOfGraph;
    if(!n) break;
    g = &n->container;
  }
  return g;
}

bool Graph::isChildOfGraph(const Graph& G) const {
  const Graph* g=this;
  for(;;) {
    const Node* n=g->isNodeOfGraph;
    if(!n) break;
    g = &n->container;
    if(g==&G) return true;
  }
  return false;
}

bool Graph::checkConsistency() const {
  uint idx=0;

#if 0 //this is expensive: fill all the parentsOf lists
  NodeL ALL = getAllNodesRecursively();
  if(!isDoubleListed) {
    for(Node* n: ALL) n->children.clear();
    for(Node* n: ALL) for(Node* p:n->parents) p->children.append(n);
  }
  for(Node* n: ALL) CHECK_EQ(n->numChildren, n->children.N, "");
#endif

  for(Node* node: *this) {
    CHECK_EQ(&node->container, this, "");
    if(isIndexed) CHECK_EQ(node->index, idx, "");
    if(isDoubleLinked) {
      CHECK_EQ(node->numChildren, node->children.N, "");
#ifndef RAI_NOCHECK
      for(Node* j: node->parents)  CHECK(j->children.findValue(node) != -1, "");
      for(Node* j: node->children) CHECK(j->parents.findValue(node) != -1, "");
#endif
    }
    for(Node* parent: node->parents) if(&parent->container!=this) {
        //check that parent is contained in a super-graph of this
        const Graph* parentGraph = this;
        const Node* parentGraphNode;
        while(&parent->container!=parentGraph) {
          //we need to descend one more
          parentGraphNode = parentGraph->isNodeOfGraph;
          CHECK(parentGraphNode, "there is no more supergraph to find the parent");
          parentGraph = &parentGraphNode->container;
        }
        //check sorting
        //      CHECK(parent->index < parentGraphNode->index,"subnode refers to parent that sorts below the subgraph");
      } else {
        //      CHECK(parent->index < node->index,"node refers to parent that sorts below the node");
      }
    if(node->isGraph()) {
      Graph& G = node->graph();
      CHECK_EQ(G.isNodeOfGraph, node, "");
      G.checkConsistency();
    }
    idx++;
  }
  return true;
}

uint Graph::index(bool subKVG, uint start) {
  uint idx=start;
  for(Node* it: list()) {
    it->index=idx;
    idx++;
    if(it->isGraph()) {
      Graph& G=it->graph();
      if(subKVG) idx = G.index(true, idx);
      else G.index(false, 0);
    }
  }
  isIndexed=true;
  return idx;
}

bool operator==(const Graph& A, const Graph& B) {
  if(A.N!=B.N) return false;
  for(uint i=0; i<A.N; i++) {
    Node* a = A(i), *b = B(i);
    if(a->index!=b->index) return false;
    if(a->key!=b->key) return false;
    if(a->parents.N!=b->parents.N) return false;
    for(uint j=0; j<a->parents.N; j++) if(a->parents(j)->index!=b->parents(j)->index) return false;
    if(a->type!=b->type) return false;
    if(!a->hasEqualValue(b)) return false;
  }
  return true;
}

//===========================================================================

NodeL neighbors(Node* it) {
  NodeL N;
  for(Node* e:it->children) {
    for(Node* n:e->parents) if(n!=it) N.setAppend(n);
  }
  return N;
}

int distance(NodeL A, NodeL B) {
  CHECK(A.N, "");
  CHECK(B.N, "");
  Graph& G=A.first()->container;
  CHECK_EQ(&B.first()->container, &G, "");

  boolA doneA(G.N), doneB(G.N);
  doneA.setZero();
  doneB.setZero();
  NodeL fringeA = A;
  NodeL fringeB = B;
  int D=0;
  for(Node* a:fringeA) doneA(a->index) = true;
  for(Node* b:fringeB) { if(doneA(b->index)) return D; doneB(b->index) = true; }
  for(;;) {
    D++;
    NodeL newA;
    for(Node* a:fringeA) {
      NodeL neighA = neighbors(a);
      for(Node* n:neighA) {
        if(doneB(n->index)) return D;
        if(!doneA(n->index)) { newA.append(n); doneA(n->index)=true; }
      }
    }
    D++;
    NodeL newB;
    for(Node* b:fringeB) {
      NodeL neighB = neighbors(b);
      for(Node* n:neighB) {
        if(doneA(n->index)) return D;
        if(!doneB(n->index)) { newB.append(n); doneB(n->index)=true; }
      }
    }
    if(!newA.N && !newB.N) break; //failure
    fringeA = newA;
    fringeB = newB;
  }
  return -1;
}

//===========================================================================
//
// global singleton TypeRegistrationSpace
//

Singleton<Graph> parameterGraph;

Mutex::TypedToken<Graph> getParameters(){
  return parameterGraph();
}

void initParameters(int _argc, char*_argv[]){
  static bool wasInitialized=false;
  if(wasInitialized) return;
  wasInitialized=true;

  auto P = parameterGraph();
  //-- parse cmd line arguments into graph
  StringA tags;
  for(int n=1; n<argc; n++) {
    if(rai::argv[n][0]=='-') {
      rai::String key(rai::argv[n]+1);
      if(n+1<rai::argc && rai::argv[n+1][0]!='-') {
        rai::String value;
        value <<':' <<rai::argv[n+1];
        P->readNode(value, tags, key, false, false);
        n++;
      } else {
        P->newNode<bool>(key, {}, true);
      }
    } else {
      RAI_MSG("non-parsed cmd line argument:" <<rai::argv[n]);
    }
  }

  //-- append 'rai.cfg'
  rai::String cfgFileName="rai.cfg";
  if(P->findNode("cfg")) cfgFileName = P->get<rai::String>("cfg");
  LOG(3) <<"opening config file '" <<cfgFileName <<"'";
  ifstream fil;
  fil.open(cfgFileName);
  if(fil.good()) {
    fil >>P();
  } else {
    LOG(3) <<" - failed";
  }

  LOG(1) <<"** parsed parameters:\n" <<P() <<'\n';
}

} //namespace

//  registry()->findNodeOfType(type, {key});
//  if(n) {
//    n->copyValueInto(data);
//    return true;
//  } else {
//    n = registry()->findNode({key});
//    if(n && n->isOfType<double>()) {
//      if(type==typeid(int)) { *((int*)data) = (int)n->get<double>(); return true; }
//      if(type==typeid(uint)) { *((uint*)data) = (uint)n->get<double>(); return true; }
//      if(type==typeid(bool)) { *((bool*)data) = (bool)n->get<double>(); return true; }
//    }
//    if(n && n->isOfType<rai::String>()) {
//      NIY;
//      //      n->get<rai::String>() >>x;
//    }
//  }
//  return false;
//}

//===========================================================================

RUN_ON_INIT_BEGIN(graph)
rai::NodeL::memMove=true;
rai::GraphEditCallbackL::memMove=true;
RUN_ON_INIT_END(graph)
