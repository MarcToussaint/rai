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


#include <map>

#include "util_t.h"
#include "array_t.h"
#include "graph.h"
#include "registry.h"

#define DEBUG(x)

NodeL& NoNodeL=*((NodeL*)NULL);
Graph& NoGraph=*((Graph*)NULL);

//===========================================================================

struct ParseInfo{
  Node *it;
  istream::pos_type beg,end;
  istream::pos_type err_beg, err_end;
  enum Error{ good=0, unknownParent };
  void write(ostream& os) const{ os <<'<' <<beg <<',' <<end <<'>'; }
};
stdOutPipe(ParseInfo)

//===========================================================================
//
//  Node methods
//

Node::Node(Graph& _container)
  : container(_container){
  if(&container!=&NoGraph){
    index=container.N;
    container.NodeL::append(this);
  }else{
    HALT("don't do that anymore!");	
    index=(uint)(-1);
  }
}

Node::Node(Graph& _container, const StringA& _keys, const NodeL& _parents)
  : container(_container), keys(_keys), parents(_parents){
  CHECK(&container!=&NoGraph, "you gave me a NoGraph container");
  index=container.N;
  container.NodeL::append(this);
  for(Node *i: parents){
    CHECK(i,"you gave me a NULL parent");
    i->parentOf.append(this);
  }
}

Node::~Node() {
  for(Node *i: parents) i->parentOf.removeValue(this);
  for(Node *i: parentOf) i->parents.removeValue(this);
  container.removeValue(this);
  container.index();
}

bool Node::matches(const char *key){
  for(const MT::String& k:keys) if(k==key) return true;
  return false;
}

bool Node::matches(const StringA &query_keys) {
  for(const MT::String& k:query_keys) {
    if(!matches(k)) return false;
  }
  return true;
}

void Node::write(std::ostream& os) const {
  //-- write keys
  keys.write(os, " ", "", "\0\0");
  
  //-- write parents
  if(parents.N) {
    //    if(keys.N) os <<' ';
    os <<'(';
    for_list(Node, it, parents) {
      if(it_COUNT) os <<' ';
      if(it->keys.N){
        os <<it->keys.last();
      }else{  //relative numerical reference
        os <<(int)it->index - (int)index;
      }
    }
    os <<')';
  }
  
  //-- write value
  if(!hasValue()) return;
  if(getValueType()==typeid(Graph)) {
    os <<" {";
    getValue<Graph>()->write(os, " ");
    os <<" }";
  } else if(getValueType()==typeid(NodeL)) {
    os <<"=(";
    for(Node *it: (*getValue<NodeL>())) os <<' ' <<it->keys.last();
    os <<" )";
  } else if(getValueType()==typeid(MT::String)) {
    os <<"=\"" <<*getValue<MT::String>() <<'"';
  } else if(getValueType()==typeid(MT::FileToken)) {
    os <<"='" <<getValue<MT::FileToken>()->name <<'\'';
  } else if(getValueType()==typeid(arr)) {
    os <<'='; getValue<arr>()->write(os, NULL, NULL, "[]");
  } else if(getValueType()==typeid(double)) {
    os <<'=' <<*getValue<double>();
  } else if(getValueType()==typeid(bool)) {
    if(*getValue<bool>()) os<<','; else os <<'!';
  } else {
    Node *it = reg_findType(getValueType().name());
    if(it && it->keys.N>1) {
      os <<" = <" <<it->keys(1) <<' ';
      writeValue(os);
      os <<'>';
    } else {
      os <<" = < ";
      writeValue(os);
      os <<'>';
    }
  }
}

Graph Node::ParentOf(){
  Graph G;
  G.isReferringToNodesOf = &container;
  G.NodeL::operator=(parentOf);
  return G;
}

Node *readNode(Graph& containingGraph, std::istream& is, bool verbose, bool parseInfo, MT::String prefixedKey=MT::String()) {
  MT::String str;
  StringA keys;
  NodeL parents;
  Node *item=NULL;

  istream::pos_type is_startPos=is.tellg();
  istream::pos_type is_lastPos=is.tellg();

  if(verbose) { cout <<"\nITEM (line="<<MT::lineCount <<")"; }

#define PARSERR(x) { cerr <<"[[error in parsing Graph file (line=" <<MT::lineCount <<"):\n"\
                          <<"  item keys=" <<keys <<"\n  error=" <<x <<"]]"; is.clear(); }
  
  //-- read keys
  if(!prefixedKey.N){
    MT::skip(is," \t\n\r");
    for(;;) {
      if(!str.read(is, " \t", " \t\n\r,;([{}=", false)) break;
      is_lastPos=is.tellg();
      keys.append(str);
    }
    //if(!keys.N) return false;
  }else{
    keys.append(prefixedKey);
  }
  
  if(verbose) { cout <<" keys:" <<keys <<flush; }
  
  //-- read parents
  char c=MT::getNextChar(is," \t"); //don't skip new lines
  if(c=='(') {
    for(uint j=0;; j++) {
      if(!str.read(is, " \t\n\r,", " \t\n\r,)", false)) break;
      is_lastPos=is.tellg();
      Node *e=containingGraph.getNode(str);
      if(e) { //sucessfully found
        parents.append(e);
      } else { //this element is not known!!
        int rel=0;
        str >>rel;
        if(rel<0 && (int)containingGraph.N+rel>=0){
          e=containingGraph(containingGraph.N+rel);
          parents.append(e);
        }else{
          PARSERR("unknown " <<j <<". parent '" <<str <<"'");
          MT::skip(is, NULL, ")", false);
        }
      }
    }
    MT::parse(is, ")");
    is_lastPos=is.tellg();
    c=MT::getNextChar(is," \t");
  }
  
  if(verbose) { cout <<" parents:"; if(!parents.N) cout <<"none"; else listWrite(parents,cout," ","()"); cout <<flush; }
  
  //-- read value
  if(c=='=' || c=='{' || c=='[' || c=='<' || c=='!') {
    if(c=='=') c=MT::getNextChar(is," \t");
    is_lastPos=is.tellg();
    if((c>='a' && c<='z') || (c>='A' && c<='Z')) { //MT::String or boolean
      is.putback(c);
      str.read(is, "", " \n\r\t,;}", false);
      if(str=="true") item = new Node_typed<bool>(containingGraph, keys, parents, new bool(true), true);
      else if(str=="false") item = new Node_typed<bool>(containingGraph, keys, parents, new bool(false), true);
      else item = new Node_typed<MT::String>(containingGraph, keys, parents, new MT::String(str), true);
    } else if(MT::contains("-.0123456789", c)) {  //single double
      is.putback(c);
      double d;
      try { is >>d; } catch(...) PARSERR("can't parse double");
      item = new Node_typed<double>(containingGraph, keys, parents, new double(d), true);
    } else switch(c) {
      case '!': { //boolean false
        item = new Node_typed<bool>(containingGraph, keys, parents, new bool(false), true);
      } break;
      case '\'': { //MT::FileToken
        str.read(is, "", "\'", true);
        MT::FileToken *f = new MT::FileToken(str, false);
        try{
          f->getIs(); //creates the ifstream and might throw an error
          item = new Node_typed<MT::FileToken>(containingGraph, keys, parents, f, true);
        } catch(...){
          PARSERR("kvg indicates file which does not exist -> converting to string!");
          item = new Node_typed<MT::String>(containingGraph, keys, parents, new MT::String(str), true);
          delete f;
        }
      } break;
      case '\"': { //MT::String
        str.read(is, "", "\"", true);
        item = new Node_typed<MT::String>(containingGraph, keys, parents, new MT::String(str), true);
      } break;
      case '[': { //arr
        is.putback(c);
        arr reals;
        is >>reals;
        item = new Node_typed<arr>(containingGraph, keys, parents, new arr(reals), true);
      } break;
      case '<': { //any type parser
        str.read(is, " \t", " \t\n\r()`-=~!@#$%^&*()+[]{};'\\:|,./<>?", false);
        //      str.read(is, " \t", " \t\n\r()`1234567890-=~!@#$%^&*()_+[]{};'\\:|,./<>?", false);
        item = readTypeIntoNode(containingGraph, str, is);
        if(!item) {
          is.clear();
          MT_MSG("could not parse value of type '" <<str <<"' -- no such type has been registered");
          str.read(is,"",">",false);
          MT_MSG("ignoring: '"<<str<<"'");
        } else {
          item->keys = keys;
          item->parents = parents;
        }
        MT::parse(is, ">");
      } break;
      case '{': { // Graph (e.g., attribute list)
        Graph *subList = new Graph;
        item = new Node_typed<Graph>(containingGraph, keys, parents, subList, true);
        subList->read(is);
        MT::parse(is, "}");
      } break;
      case '(': { // referring Graph
        Graph *refs = new Graph;
        refs->isReferringToNodesOf = &containingGraph;
        for(uint j=0;; j++) {
          str.read(is, " , ", " , )", false);
          if(!str.N) break;
          Node *e=containingGraph.getNode(str);
          if(e) { //sucessfully found
            refs->NodeL::append(e);
          } else { //this element is not known!!
            HALT("line:" <<MT::lineCount <<" reading item '" <<keys <<"': unknown "
                 <<j <<"th linked element '" <<str <<"'"); //DON'T DO THIS YET
          }
        }
        MT::parse(is, ")");
        item = new Node_typed<Graph>(containingGraph, keys, parents, refs, true);
      } break;
      default: { //error
        is.putback(c);
        PARSERR("unknown value indicator '" <<c <<"'");
        return NULL;
      }
    }
    is_lastPos=is.tellg();
  } else { //no '=' or '{' -> boolean
    is.putback(c);
    item = new Node_typed<bool>(containingGraph, keys, parents, new bool(true), true);
  }

#undef PARSERR
  if(parseInfo && item){
    item->container.getParseInfo(item).beg=is_startPos;
    item->container.getParseInfo(item).end=is_lastPos;
  }

  if(verbose) {
    if(item) { cout <<" value:"; item->writeValue(cout); cout <<" FULL:"; item->write(cout); cout <<endl; }
    else { cout <<"FAILED" <<endl; }
  }
  
  if(!item){
    cout <<"FAILED reading item with keys ";
    keys.write(cout, " ", NULL, "()");
    cout <<" and parents ";
    listWrite(parents,cout," ","()");
    cout <<endl;
  }
  
  //eat the next , or ;
  c=MT::getNextChar(is," \n\r\t");
  if(c==',' || c==';') {} else is.putback(c);
  
  return item;
}


NodeInitializer::NodeInitializer(const char* key){
  it = new Node_typed<bool>(G, NULL, false);
  it->keys.append(STRING(key));
}


//===========================================================================
//
//  Graph methods
//

struct sKeyValueGraph {
  //  std::map<std::string, Node*> keyMap;
};

Graph::Graph():s(NULL), isReferringToNodesOf(NULL), isNodeOfParentGraph(NULL) {
}

Graph::Graph(const char* filename):s(NULL), isReferringToNodesOf(NULL), isNodeOfParentGraph(NULL) {
  read(MT::FileToken(filename).getIs());
}

Graph::Graph(istream& is):s(NULL), isReferringToNodesOf(NULL), isNodeOfParentGraph(NULL) {
  read(is);
}

Graph::Graph(const std::map<std::string, std::string>& dict):s(NULL), isReferringToNodesOf(NULL), isNodeOfParentGraph(NULL) {
  appendDict(dict);
}

Graph::Graph(std::initializer_list<NodeInitializer> list):s(NULL), isReferringToNodesOf(NULL), isNodeOfParentGraph(NULL)  {
  for(const NodeInitializer& ic:list){
    Node *clone = ic.it->newClone(*this); //this appends sequentially clones of all items to 'this'
    for(const MT::String& s:ic.parents){
      Node *p = getNode(s);
      CHECK(p,"parent " <<p <<" of " <<*clone <<" does not exist!");
      clone->parents.append(p);
      p->parentOf.append(clone);
    }
  }
}

Graph::Graph(const Graph& G):s(NULL), isReferringToNodesOf(NULL), isNodeOfParentGraph(NULL) {
  *this = G;
}

Graph::~Graph() {
  clear();
  if(isNodeOfParentGraph){
    Node_typed<Graph>* it=dynamic_cast<Node_typed<Graph>*>(isNodeOfParentGraph);
    CHECK(it,"");
    it->value=NULL;
    it->ownsValue=false;
  }
}

void Graph::clear() {
  if(!isReferringToNodesOf){
    checkConsistency();
    while(N) delete last();
    checkConsistency();
  }else{
    NodeL::clear();
  }
}

Node *Graph::append(const uintA& parentIdxs) {
  NodeL parents(parentIdxs.N);
  for(uint i=0;i<parentIdxs.N; i++) parents(i) = NodeL::elem(parentIdxs(i));
  return append<int>({STRING(NodeL::N)}, parents, NULL, false);
}

void Graph::appendDict(const std::map<std::string, std::string>& dict){
  for(const std::pair<std::string,std::string>& p:dict){
    Node *it = readNode(*this, STRING('='<<p.second), false, false, MT::String(p.first));
    if(!it) MT_MSG("failed to read dict entry <" <<p.first <<',' <<p.second <<'>');
  }
}

Node* Graph::getNode(const char *key) const {
  for(Node *it: (*this)) if(it->matches(key)) return it;
  //    for(const MT::String& k:it->keys) if(k==key) return it;
  if(isNodeOfParentGraph) return isNodeOfParentGraph->container.getNode(key);
//  MT_MSG("no node with key '"<<key <<"' found");
  return NULL;
}

Node* Graph::getNode(const char *key1, const char *key2) const {
  for(Node *it: (*this)) {
    for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key1) {
      for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key2)
        return it;
    }
  }
//  MT_MSG("no node with keys '"<<key1 <<' ' <<key2 <<"' found");
  return NULL;
}

Node* Graph::getNode(const StringA &keys) const {
  for(Node *it: (*this)) if(it->matches(keys)) return it;
  if(isNodeOfParentGraph) return isNodeOfParentGraph->container.getNode(keys);
//  MT_MSG("no node with keys '"<<keys <<"' found");
  return NULL;
}

NodeL Graph::getNodes(const char* key) const {
  NodeL ret;
  for(Node *it: (*this)) if(it->matches(key)) ret.append(it);
  return ret;
}

Node* Graph::getChild(Node *p1, Node *p2) const{
  if(p1->parentOf.N < p2->parentOf.N){
    for(Node *i:p1->parentOf){
      if(p2->parentOf.findValue(i)!=-1) return i;
    }
  }else{
    for(Node *i:p2->parentOf){
      if(p1->parentOf.findValue(i)!=-1) return i;
    }
  }
  return NULL;
}

NodeL Graph::getNodesOfDegree(uint deg) {
  NodeL ret;
  for(Node *it: (*this)) {
    if(it->parents.N==deg) ret.append(it);
  }
  return ret;
}


NodeL Graph::getTypedNodes(const char* key, const std::type_info& type) {
  NodeL ret;
  for(Node *it: (*this)) if(it->getValueType()==type) {
    if(!key) ret.NodeL::append(it);
    else for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) {
      ret.append(it);
      break;
    }
  }
  return ret;
}

Node* Graph::merge(Node *m){
  NodeL KVG = getTypedNodes(m->keys(0), m->getValueType());
  //CHECK(KVG.N<=1, "can't merge into multiple items yet");
  Node *it=NULL;
  if(KVG.N) it=KVG.elem(0);
  if(it){
    CHECK(m->getValueType()==it->getValueType(), "can't merge items of different types!");
    if(it->getValueType()==typeid(Graph)){ //merge the KVGs
      it->getValue<Graph>()->merge(*m->getValue<Graph>());
    }else{ //overwrite the value
      it->takeoverValue(m);
    }
    if(&m->container==this) delete m;
  }else{ //nothing to merge, append
    if(&m->container!=this){
      Node *it = m->newClone(*this);
      for(uint i=0;i<it->parents.N;i++){
        it->parents(i) = elem(it->parents(i)->index);
        it->parents(i)->parentOf.append(it);
      }
    }
    return m;
  }
  return NULL;
}

void Graph::copy(const Graph& G, Graph* becomeSubgraphOfContainer){
  DEBUG(G.checkConsistency();)

  //-- first delete existing items
  if(!isReferringToNodesOf){ while(N) delete last(); } // listDelete(*this);

  //-- make this become a subgraph
  if(becomeSubgraphOfContainer){ //CHECK that this is also a subgraph of the same container..
    if(!isNodeOfParentGraph){
      Node *Git = G.isNodeOfParentGraph;
      if(Git)
        new Node_typed<Graph>(*becomeSubgraphOfContainer, Git->keys, Git->parents, this, true);
      else
        new Node_typed<Graph>(*becomeSubgraphOfContainer, {}, {}, this, true);
    }else{
      CHECK(&isNodeOfParentGraph->container==becomeSubgraphOfContainer,"is already subgraph of another container!");
    }
  }

  //-- first, just clone items with their values -- 'parents' still point to the origin items
  for(Node *it:G){
    if(it->getValueType()==typeid(Graph) && it->getValue<Graph>()!=NULL){
      // why we can't copy the subgraph yet:
      // copying the subgraph would require to fully rewire the subgraph (code below)
      // but if the subgraph refers to parents of this graph that are not create yet, requiring will fail
      // therefore we just insert an empty graph here; we then copy the subgraph once all items are created
      new Node_typed<Graph>(*this, it->keys, it->parents, new Graph(), true);
    }else{
      it->newClone(*this); //this appends sequentially clones of all items to 'this'
    }
  }

  //-- the new items are not parent of anybody yet
  for(Node *it:*this) CHECK(it->parentOf.N==0,"");

  //-- now copy subgraphs
  for(Node *it:*this) if(it->getValueType()==typeid(Graph) && it->getValue<Graph>()!=NULL){
    it->graph().isNodeOfParentGraph = it;
    it->graph().copy(G.elem(it->index)->graph(), NULL); //you can only call the operator= AFTER assigning isNodeOfParentGraph
  }

  //-- now rewire links
  for(Node *it:*this){
    for(uint i=0;i<it->parents.N;i++){
      Node *p=it->parents(i); //the parent in the origin graph
      const Graph *newg=this, *oldg=&G;
      while(&p->container!=oldg){  //find the container while iterating backward also in the newG
        CHECK(oldg->isNodeOfParentGraph,"");
        newg = &newg->isNodeOfParentGraph->container;
        oldg = &oldg->isNodeOfParentGraph->container;
      }
      CHECK(newg->N==oldg->N,"different size!!\n" <<*newg <<"**\n" <<*oldg);
      CHECK(p==oldg->elem(p->index),""); //we found the parent in oldg
      p->parentOf.removeValue(it);  //origin items is not parent of copy
      p = newg->elem(p->index);     //the true parent in the new graph
      p->parentOf.append(it);       //connect both ways
      it->parents(i)=p;
    }
  }

  DEBUG(this->checkConsistency();)
  DEBUG(G.checkConsistency();)
}

void Graph::read(std::istream& is, bool parseInfo) {
  if(parseInfo) getParseInfo(NULL).beg=is.tellg();
  for(;;) {
    char c=MT::peerNextChar(is, " \n\r\t,");
    if(!is.good() || c=='}') { is.clear(); break; }
    Node *it = readNode(*this, is, false, parseInfo);
    if(!it) break;
    if(it->keys.N==1 && it->keys.last()=="Include"){
      read(it->getValue<MT::FileToken>()->getIs(true));
      delete it;
    }else
    if(it->keys.N==1 && it->keys.last()=="ChDir"){
      it->getValue<MT::FileToken>()->changeDir();
    }
  }
  if(parseInfo) getParseInfo(NULL).end=is.tellg();
  //-- merge all Merge keys
  NodeL merges = getNodes("Merge");
  for(Node *m:merges){
    m->keys.remove(0);
    merge(m);
  }
  for(uint i=N;i--;){
    Node *it=elem(i);
    if(it->keys.N==1 && it->keys(0)=="ChDir"){
      it->getValue<MT::FileToken>()->unchangeDir();
      delete it;
    }
  }
}

void Graph::write(std::ostream& os, const char *ELEMSEP, const char *delim) const {
  if(delim) os <<delim[0];
  for(uint i=0; i<N; i++) { if(i) os <<ELEMSEP;  if(elem(i)) elem(i)->write(os); else os <<"<NULL>"; }
  if(delim) os <<delim[1] <<std::flush;
}

void Graph::writeParseInfo(std::ostream& os) {
  os <<"GRAPH " <<getParseInfo(NULL) <<endl;
  for(Node *n:*this)
    os <<"NODE '" <<*n <<"' " <<getParseInfo(n) <<endl;
}

void Graph::writeDot(std::ostream& os, bool withoutHeader, bool defaultEdges, int nodesOrEdges) {
  if(!withoutHeader){
    if(defaultEdges) os <<"digraph G{" <<endl;
    else             os <<"graph G{" <<endl;
    os <<"graph [ rankdir=\"LR\", ranksep=0.05 ];" <<endl;
    os <<"node [ fontsize=9, width=.3, height=.3 ];" <<endl;
    os <<"edge [ arrowtail=dot, arrowsize=.5, fontsize=6 ];" <<endl;
    index(true);
  }
  for(Node *it: list()) {
    MT::String label, shape("shape=ellipse");
    if(it->keys.N){
      label <<"label=\"";
      bool newline=false;
      for(MT::String& k:it->keys){
        if(k=="box") shape="shape=box";
        else{
          if(newline) label <<'\n';
          label <<k;
          newline=true;
        }
      }
      label <<"\" ";
    }else if(it->parents.N){
      label <<"label=\"(" <<it->parents(0)->keys.last();
      for(uint i=1;i<it->parents.N;i++) label <<' ' <<it->parents(i)->keys.last();
      label <<")\" ";
    }

    if(defaultEdges && it->parents.N==2){ //an edge
      os <<it->parents(0)->index <<" -> " <<it->parents(1)->index <<" [ " <<label <<"];" <<endl;
    }else{
      if(it->getValueType()==typeid(Graph)){
        os <<"subgraph cluster_" <<it->index <<" { " <<label /*<<" rank=same"*/ <<endl;
        it->getValue<Graph>()->writeDot(os, true, defaultEdges, +1);
        os <<"}" <<endl;
        it->getValue<Graph>()->writeDot(os, true, defaultEdges, -1);
      }else{//normal item
        if(nodesOrEdges>=0){
          os <<it->index <<" [ " <<label <<shape <<" ];" <<endl;
        }
        if(nodesOrEdges<=0){
          for_list(Node, pa, it->parents) {
            if(pa->index<it->index)
              os <<pa->index <<" -- " <<it->index <<" [ ";
            else
              os <<it->index <<" -- " <<pa->index <<" [ ";
            os <<"label=" <<pa_COUNT;
            os <<" ];" <<endl;
          }
        }
      }
    }
  }
  if(!withoutHeader){
    os <<"}" <<endl;
    index(false);
  }
}

void Graph::sortByDotOrder() {
  uintA perm;
  perm.setStraightPerm(N);
  for_list(Node, it, list()) {
    if(it->getValueType()==typeid(Graph)) {
      double *order = it->getValue<Graph>()->getValue<double>("dot_order");
      if(!order) { MT_MSG("doesn't have dot_order attribute"); return; }
      perm(it_COUNT) = (uint)*order;
    }
  }
  permuteInv(perm);
  for_list(Node, it2, list()) it2->index=it2_COUNT;
}

ParseInfo& Graph::getParseInfo(Node* it){
  if(pi.N!=N+1){
    listResizeCopy(pi, N+1);
    pi(0)->it=NULL;
    for(uint i=1;i<pi.N;i++) pi(i)->it=elem(i-1);
  }
  if(!it) return *pi(0);
  return *pi(it->index+1);
}

bool Graph::checkConsistency() const{
  uint idx=0;
  for(Node *it: *this){
    CHECK_EQ(&it->container, this, "");
    CHECK_EQ(it->index, idx, "");
    for(Node *j: it->parents)  CHECK(j->parentOf.findValue(it) != -1,"");
    for(Node *j: it->parentOf) CHECK(j->parents.findValue(it) != -1,"");
    for(Node *parent: it->parents) if(&parent->container!=this){
      //check that parent is contained in a super-graph of this
      const Graph *parentGraph = this;
      const Node *parentGraphNode;
      while(&parent->container!=parentGraph){
        //wee need to descend one more
        parentGraphNode = parentGraph->isNodeOfParentGraph;
        CHECK(parentGraphNode,"there is no more supergraph to find the parent");
        parentGraph = &parentGraphNode->container;
      }
      //check sorting
//      CHECK(parent->index < parentGraphNode->index,"subitem refers to parent that sorts below the subgraph");
    }else{
      CHECK(parent->index < it->index,"item refers to parent that sorts below the item");
    }
    if(it->getValueType()==typeid(Graph) && it->getValue<Graph>()){
      Graph& G = it->graph();
      CHECK(G.isNodeOfParentGraph==it,"");
      if(!G.isReferringToNodesOf) G.checkConsistency();
    }
    idx++;
  }
  return true;
}

uint Graph::index(bool subKVG, uint start){
  uint idx=start;
  for(Node *it: list()){
    it->index=idx;
    idx++;
    if(it->getValueType()==typeid(Graph) && it->getValue<Graph>()){
      Graph& G=it->graph();
      if(!G.isReferringToNodesOf){
        if(subKVG) idx = G.index(true, idx);
        else G.index(false, 0);
      }
    }
  }
  return idx;
}

//===========================================================================

NodeL neighbors(Node* it){
  NodeL N;
  for(Node *e:it->parentOf){
    for(Node *n:e->parents) if(n!=it) N.setAppend(n);
  }
  return N;
}

//===========================================================================

RUN_ON_INIT_BEGIN(graph)
NodeL::memMove=true;
RUN_ON_INIT_END(graph)
