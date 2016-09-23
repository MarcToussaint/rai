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

#include "util.tpp"
#include "array.tpp"
#include "graph.h"
#include "registry.h"

#define DEBUG(x)

NodeL& NoNodeL=*((NodeL*)NULL);
Graph& NoGraph=*((Graph*)NULL);

//===========================================================================
//
// annotations to a node while parting; can be used for highlighting and error messages
//

struct ParseInfo{
  Node *node;
  istream::pos_type beg,end;
  istream::pos_type err_beg, err_end;
  istream::pos_type keys_beg, keys_end;
  istream::pos_type parents_beg, parents_end;
  istream::pos_type value_beg, value_end;
  enum Error{ good=0, unknownParent };
  ParseInfo():node(NULL){}
  void write(ostream& os) const{ os <<'<' <<beg <<',' <<end <<'>'; }
};
stdOutPipe(ParseInfo)


//===========================================================================
//
//  Node methods
//

Node::Node(const std::type_info& _type, void* _value_ptr, Graph& _container)
  : type(_type), value_ptr(_value_ptr), container(_container){
  CHECK(&container!=&NoGraph, "don't do that anymore!");
  index=container.N;
  container.NodeL::append(this);
}

Node::Node(const std::type_info& _type, void* _value_ptr, Graph& _container, const StringA& _keys, const NodeL& _parents)
  : type(_type), value_ptr(_value_ptr), container(_container), keys(_keys), parents(_parents){
  CHECK(&container!=&NoGraph, "This is a NoGraph (NULL) -- don't do that anymore!");
  index=container.N;
  container.NodeL::append(this);
  if(parents.N) for(Node *i: parents){
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
  for(const mlr::String& k:keys) if(k==key) return true;
  return false;
}

bool Node::matches(const StringA &query_keys) {
  for(const mlr::String& k:query_keys) {
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
  if(isGraph()) {
    os <<" {";
    graph().write(os, " ");
    os <<" }";
  } else if(isOfType<NodeL>()) {
    os <<"=(";
    for(Node *it: (*getValue<NodeL>())) os <<' ' <<it->keys.last();
    os <<" )";
  } else if(isOfType<mlr::String>()) {
    os <<"=\"" <<*getValue<mlr::String>() <<'"';
  } else if(isOfType<mlr::FileToken>()) {
    os <<"='" <<getValue<mlr::FileToken>()->name <<'\'';
  } else if(isOfType<arr>()) {
    os <<'='; getValue<arr>()->write(os, NULL, NULL, "[]");
  } else if(isOfType<double>()) {
    os <<'=' <<*getValue<double>();
  } else if(isOfType<bool>()) {
    if(*getValue<bool>()) os<<','; else os <<'!';
  } else if(isOfType<Type*>()) {
    os <<" = "; get<Type*>()->write(os);
  } else {
    Node *it = reg_findType(type.name());
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

Nod::Nod(const char* key){
  n = new Node_typed<bool>(G, true);
  n->keys.append(STRING(key));
}

Nod::Nod(const char* key, const char* stringValue){
  n = new Node_typed<mlr::String>(G, STRING(stringValue));
  n->keys.append(STRING(key));
}



//===========================================================================
//
//  Graph methods
//

Graph::Graph():isNodeOfParentGraph(NULL) {
}

Graph::Graph(const char* filename):isNodeOfParentGraph(NULL) {
  read(mlr::FileToken(filename).getIs());
}

Graph::Graph(istream& is):isNodeOfParentGraph(NULL) {
  read(is);
}

Graph::Graph(const std::map<std::string, std::string>& dict):isNodeOfParentGraph(NULL) {
  appendDict(dict);
}

Graph::Graph(std::initializer_list<Nod> list):isNodeOfParentGraph(NULL)  {
  for(const Nod& ni:list) append(ni);
}

Graph::Graph(const Graph& G):isNodeOfParentGraph(NULL) {
  *this = G;
}

Graph::~Graph() {
  clear();
}

void Graph::clear() {
  while(N) delete last();
}

Graph& Graph::append(const Nod& ni){
  Node *clone = ni.n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
  for(const mlr::String& s:ni.parents){
    Node *p = getNode(s);
    CHECK(p,"parent " <<p <<" of " <<*clone <<" does not exist!");
    clone->parents.append(p);
    p->parentOf.append(clone);
  }
  return *this;
}

Node_typed<Graph>* Graph::newSubgraph(const StringA& keys, const NodeL& parents, const Graph& x){
  Node_typed<Graph>* n = new Node_typed<Graph>(*this, keys, parents, Graph());
  DEBUG( CHECK(n->value.isNodeOfParentGraph && &n->value.isNodeOfParentGraph->container==this,"") )
  if(&x) n->value.copy(x);
  return n;
}

Node_typed<int>* Graph::append(const uintA& parentIdxs) {
  NodeL parents(parentIdxs.N);
  for(uint i=0;i<parentIdxs.N; i++) parents(i) = NodeL::elem(parentIdxs(i));
  return append<int>({STRING(NodeL::N)}, parents, 0);
}

void Graph::appendDict(const std::map<std::string, std::string>& dict){
  for(const std::pair<std::string,std::string>& p:dict){
    Node *n = readNode(STRING('='<<p.second), false, false, mlr::String(p.first));
    if(!n) MLR_MSG("failed to read dict entry <" <<p.first <<',' <<p.second <<'>');
  }
}

Node* Graph::findNode(const StringA& keys, bool recurseUp, bool recurseDown) const {
  for(Node* n: (*this)) if(n->matches(keys)) return n;
  Node* ret=NULL;
  if(recurseUp && isNodeOfParentGraph) ret = isNodeOfParentGraph->container.findNode(keys, true, false);
  if(ret) return ret;
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()){
    ret = n->graph().findNode(keys, false, true);
    if(ret) return ret;
  }
  return ret;
}

Node* Graph::findNodeOfType(const std::type_info& type, const StringA& keys, bool recurseUp, bool recurseDown) const {
  for(Node* n: (*this)) if(n->type==type && n->matches(keys)) return n;
  Node* ret=NULL;
  if(recurseUp && isNodeOfParentGraph) ret = isNodeOfParentGraph->container.findNodeOfType(type, keys, true, false);
  if(ret) return ret;
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()){
    ret = n->graph().findNodeOfType(type, keys, false, true);
    if(ret) return ret;
  }
  return ret;
}

NodeL Graph::findNodes(const StringA& keys, bool recurseUp, bool recurseDown) const {
  NodeL ret;
  for(Node *n: (*this)) if(n->matches(keys)) ret.append(n);
  if(recurseUp && isNodeOfParentGraph) ret.append( isNodeOfParentGraph->container.findNodes(keys, true, false) );
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()) ret.append( n->graph().findNodes(keys, false, true) );
  return ret;
}

NodeL Graph::findNodesOfType(const std::type_info& type, const StringA& keys, bool recurseUp, bool recurseDown) const {
  NodeL ret;
  for(Node *n: (*this)) if(n->type==type && n->matches(keys)) ret.append(n);
  if(recurseUp && isNodeOfParentGraph) ret.append( isNodeOfParentGraph->container.findNodesOfType(type, keys, true, false) );
  if(recurseDown) for(Node *n: (*this)) if(n->isGraph()) ret.append( n->graph().findNodesOfType(type, keys, false, true) );
  return ret;
}

//Node* Graph::getNode(const char *key) const {
//  for(Node *n: (*this)) if(n->matches(key)) return n;
//  if(isNodeOfParentGraph) return isNodeOfParentGraph->container.getNode(key);
//  return NULL;
//}

//Node* Graph::getNode(const StringA &keys) const {
//  for(Node *n: (*this)) if(n->matches(keys)) return n;
//  if(isNodeOfParentGraph) return isNodeOfParentGraph->container.getNode(keys);
//  return NULL;
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

Node* Graph::getEdge(Node *p1, Node *p2) const{
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

Node* Graph::getEdge(const NodeL& parents) const{
  CHECK(parents.N>0,"");
  //grap 'sparsest' parent:
  uint minSize = this->N;
  Node *sparsestParent = NULL;
  for(Node *p:parents) if(p->parentOf.N<minSize){ sparsestParent=p; minSize=p->parentOf.N; }
  if(!sparsestParent){
    for(Node *e:*this) if(e->parents==parents) return e;
  }else{
    for(Node *e:sparsestParent->parentOf) if(&e->container==this){
      if(e->parents==parents) return e;
    }
  }
  return NULL;
}

NodeL Graph::getNodesOfDegree(uint deg) {
  NodeL ret;
  for(Node *n: (*this)) if(n->parents.N==deg) ret.append(n);
  return ret;
}

Node* Graph::merge(Node *m){
  NodeL KVG = findNodesOfType(m->type, {m->keys(0)});
  //CHECK(KVG.N<=1, "can't merge into multiple nodes yet");
  Node *n=NULL;
  if(KVG.N) n=KVG.elem(0);
  CHECK(n!=m,"how is this possible?");
  if(n){
    CHECK(m->type==n->type, "can't merge nodes of different types!");
    if(n->isGraph()){ //merge the KVGs
      n->graph().merge(m->graph());
    }else{ //overwrite the value
      n->copyValue(m);
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

void Graph::copy(const Graph& G, bool appendInsteadOfClear, bool allowCopySubgraphToNonsubgraph){
  DEBUG(G.checkConsistency());

  if(!allowCopySubgraphToNonsubgraph && G.isNodeOfParentGraph){
    if(!this->isNodeOfParentGraph){
      HALT("Typically you should not copy a subgraph into a non-subgraph (or call the copy operator with a subgraph).\
           Use 'newSubgraph' instead\
           If you still want to do it you need to ensure that all node parents are declared, and then enforce it by setting 'allowCopySubgraphToNonsubgraph'");
    }
  }

  //-- first delete existing nodes
  if(!appendInsteadOfClear) clear();
  uint indexOffset=N;
  NodeL newNodes;

  //-- if either is a subgraph, ensure they're a subgraph of the same -- over restrictive!!
//  if(isNodeOfParentGraph || G.isNodeOfParentGraph){
//    CHECK(&isNodeOfParentGraph->container==&G.isNodeOfParentGraph->container,"is already subgraph of another container!");
//  }

  //-- first, just clone nodes with their values -- 'parents' still point to the origin nodes
  for(Node *n:G){
    Node *newn=NULL;
    if(n->isGraph()){
      // why we can't copy the subgraph yet:
      // copying the subgraph would require to fully rewire the subgraph (code below)
      // but if the subgraph refers to parents of this graph that are not create yet, requiring will fail
      // therefore we just insert an empty graph here; we then copy the subgraph once all nodes are created
      newn = this->newSubgraph(n->keys, n->parents);
    }else{
      newn = n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
    }
    newNodes.append(newn);
  }

  //-- the new nodes are not parent of anybody yet
  for(Node *n:newNodes) CHECK(n->parentOf.N==0,"");

  //-- now copy subgraphs
  for(Node *n:newNodes) if(n->isGraph()){
    n->graph().copy(G.elem(n->index-indexOffset)->graph()); //you can only call the operator= AFTER assigning isNodeOfParentGraph
  }

  //-- now rewire parental links
  for(Node *n:newNodes){
    for(uint i=0;i<n->parents.N;i++){
      Node *p=n->parents(i); //the parent in the origin graph
      if(&p->container==&G){ //parent is directly in G, no need for complicated search
        p->parentOf.removeValue(n);   //original parent is not parent of copy
        p = newNodes.elem(p->index);  //the true parent in the new graph
      }else{
        const Graph *newg=this, *oldg=&G;
        while(&p->container!=oldg){  //find the container while iterating backward also in the newG
          CHECK(oldg->isNodeOfParentGraph,"");
          CHECK(newg->isNodeOfParentGraph,"");
          newg = &newg->isNodeOfParentGraph->container;
          oldg = &oldg->isNodeOfParentGraph->container;
        }
        CHECK(newg->N==oldg->N,"different size!!\n" <<*newg <<"**\n" <<*oldg);
        CHECK(p==oldg->elem(p->index),""); //we found the parent in oldg
        p->parentOf.removeValue(n);   //original parent is not parent of copy
        p = newg->elem(p->index);     //the true parent in the new graph
      }
      p->parentOf.append(n);       //connect both ways
      n->parents(i)=p;
    }
  }

  DEBUG(this->checkConsistency();)
  DEBUG(G.checkConsistency();)
}

void Graph::read(std::istream& is, bool parseInfo) {
  if(parseInfo) getParseInfo(NULL).beg=is.tellg();
  for(;;) {
    DEBUG(checkConsistency();)
    char c=mlr::peerNextChar(is, " \n\r\t,");
    if(!is.good() || c=='}') { is.clear(); break; }
    Node *n = readNode(is, false, parseInfo);
    if(!n) break;
    if(n->keys.N==1 && n->keys.last()=="Include"){
      read(n->get<mlr::FileToken>().getIs(true));
      delete n;
    }else
    if(n->keys.N==1 && n->keys.last()=="ChDir"){
      n->get<mlr::FileToken>().changeDir();
    }else
    if(n->keys.N>0 && n->keys.first()=="Delete"){
      n->keys.remove(0);
      NodeL dels = getNodes(n->keys);
      for(Node* d: dels) delete d;
    }
  }
  if(parseInfo) getParseInfo(NULL).end=is.tellg();

  DEBUG(checkConsistency();)

  //-- merge all Merge keys
  NodeL merges = getNodes("Merge");
  for(Node *m:merges){
    m->keys.remove(0);
    merge(m);
  }

  DEBUG(checkConsistency();)

  //-- delete all ChDir nodes in reverse order
  for(uint i=N;i--;){
    Node *n=elem(i);
    if(n->keys.N==1 && n->keys(0)=="ChDir"){
      n->get<mlr::FileToken>().unchangeDir();
      delete n;
    }
  }
}

void writeFromStream(std::ostream& os, std::istream& is, istream::pos_type beg, istream::pos_type end){
  istream::pos_type here=is.tellg();
  is.seekg(beg);
  char c;
  for(uint i=end-beg;i--;){
    is.get(c);
    os <<c;
  }
  is.seekg(here);
}

#define PARSERR(x, pinfo) { \
  cerr <<"[[error in parsing Graph file (line=" <<mlr::lineCount <<"): " <<x <<":\n  \""; \
  writeFromStream(cerr, is, pinfo.beg, is.tellg()); \
  cerr <<"<<<\"  ]]" <<endl; \
  if(pinfo.node) cerr <<"  (node='" <<*pinfo.node <<"')" <<endl; \
  is.clear(); }

Node* Graph::readNode(std::istream& is, bool verbose, bool parseInfo, mlr::String prefixedKey) {
  mlr::String str;
  StringA keys;
  NodeL parents;

  ParseInfo pinfo;
  pinfo.beg=is.tellg();

  if(verbose) { cout <<"\nNODE (line="<<mlr::lineCount <<")"; }

  //-- read keys
  if(!prefixedKey.N){
    mlr::skip(is," \t\n\r");
    pinfo.keys_beg=is.tellg();
    for(;;) {
      if(!str.read(is, " \t", " \t\n\r,;([{}=!", false)) break;
      keys.append(str);
      pinfo.keys_end=is.tellg();
    }
  }else{
    keys.append(prefixedKey);
  }
  DEBUG(checkConsistency();)

  if(verbose) { cout <<" keys:" <<keys <<flush; }

  //-- read parents
  char c=mlr::getNextChar(is," \t"); //don't skip new lines
  if(c=='(') {
    pinfo.parents_beg=is.tellg();
    for(uint j=0;; j++) {
      if(!str.read(is, " \t\n\r,", " \t\n\r,)", false)) break;
      Node *e = this->findNode({str}, true, false); //important: recurse up
      if(e) { //sucessfully found
        parents.append(e);
        pinfo.parents_end=is.tellg();
      } else { //this element is not known!!
        int rel=0;
        str >>rel;
        if(rel<0 && (int)this->N+rel>=0){
          e=elem(this->N+rel);
          parents.append(e);
          pinfo.parents_end=is.tellg();
        }else{
          PARSERR("unknown " <<j <<". parent '" <<str <<"'", pinfo);
          mlr::skip(is, NULL, ")", false);
        }
      }
    }
    mlr::parse(is, ")");
    c=mlr::getNextChar(is," \t");
  }
  DEBUG(checkConsistency();)

  if(verbose) { cout <<" parents:"; if(!parents.N) cout <<"none"; else listWrite(parents,cout," ","()"); cout <<flush; }

  //-- read value
  Node *node=NULL;
  pinfo.value_beg=(long int)is.tellg()-1;
  if(c=='=' || c=='{' || c=='[' || c=='<' || c=='!') {
    if(c=='=') c=mlr::getNextChar(is," \t");
    if((c>='a' && c<='z') || (c>='A' && c<='Z')) { //mlr::String or boolean
      is.putback(c);
      str.read(is, "", " \n\r\t,;}", false);
      if(str=="true") node = new Node_typed<bool>(*this, keys, parents, true);
      else if(str=="false") node = new Node_typed<bool>(*this, keys, parents, false);
      else node = new Node_typed<mlr::String>(*this, keys, parents, str);
    } else if(mlr::contains("-.0123456789", c)) {  //single double
      is.putback(c);
      double d;
      try { is >>d; } catch(...) PARSERR("can't parse the double number", pinfo);
      node = new Node_typed<double>(*this, keys, parents, d);
    } else switch(c) {
      case '!': { //boolean false
        node = new Node_typed<bool>(*this, keys, parents, false);
      } break;
      case '\'': { //mlr::FileToken
        str.read(is, "", "\'", true);
        try{
//          f->getIs();
          node = new Node_typed<mlr::FileToken>(*this, keys, parents, mlr::FileToken(str, false));
          node->get<mlr::FileToken>().getIs();  //creates the ifstream and might throw an error
        } catch(...){
          delete node;
          PARSERR("file " <<str <<" does not exist -> converting to string!", pinfo);
          node = new Node_typed<mlr::String>(*this, keys, parents, str);
//          delete f;
        }
      } break;
      case '\"': { //mlr::String
        str.read(is, "", "\"", true);
        node = new Node_typed<mlr::String>(*this, keys, parents, str);
      } break;
      case '[': { //arr
        is.putback(c);
        arr reals;
        is >>reals;
        node = new Node_typed<arr>(*this, keys, parents, reals);
      } break;
      case '<': { //any type parser
        str.read(is, " \t", " \t\n\r()`-=~!@#$%^&*()+[]{};'\\:|,./<>?", false);
        //      str.read(is, " \t", " \t\n\r()`1234567890-=~!@#$%^&*()_+[]{};'\\:|,./<>?", false);
        node = readTypeIntoNode(*this, str, is);
        if(!node) {
          is.clear();
          mlr::String substr;
          substr.read(is,"",">",false);
          PARSERR("could not parse value of type '" <<str <<"' -- no such type has been registered; converting this to string: '"<<substr<<"'", pinfo);
          str = STRING('<' <<str <<' ' <<substr <<'>');
          node = new Node_typed<mlr::String>(*this, keys, parents, str);
        } else {
          node->keys = keys;
          node->parents = parents;
        }
        mlr::parse(is, ">");
      } break;
      case '{': { // sub graph
        Node_typed<Graph> *subgraph = this->newSubgraph(keys, parents);
        subgraph->value.read(is);
        mlr::parse(is, "}");
        node = subgraph;
      } break;
//      case '(': { // referring Graph
//        Graph *refs = new Graph;
//        refs->isReferringToNodesOf = this;
//        for(uint j=0;; j++) {
//          str.read(is, " , ", " , )", false);
//          if(!str.N) break;
//          Node *e = this->getNode(str);
//          if(e) { //sucessfully found
//            refs->NodeL::append(e);
//          } else { //this element is not known!!
//            HALT("line:" <<mlr::lineCount <<" reading node '" <<keys <<"': unknown "
//                 <<j <<"th linked element '" <<str <<"'"); //DON'T DO THIS YET
//          }
//        }
//        mlr::parse(is, ")");
//        node = new Node_typed<Graph*>(*this, keys, parents, refs, true);
//      } break;
      default: { //error
        is.putback(c);
        PARSERR("unknown value indicator '" <<c <<"'", pinfo);
        return NULL;
      }
    }
  } else { //no '=' or '{' -> boolean
    is.putback(c);
    node = new Node_typed<bool>(*this, keys, parents, true);
  }
  if(node) pinfo.value_end=is.tellg();
  pinfo.end=is.tellg();
  DEBUG(checkConsistency();)

  if(parseInfo && node){
    pinfo.node = node;
    node->container.getParseInfo(node) = pinfo;
  }

  if(verbose) {
    if(node) { cout <<" value:"; node->writeValue(cout); cout <<" FULL:"; node->write(cout); cout <<endl; }
    else { cout <<"FAILED" <<endl; }
  }

  if(!node){
    cerr <<"FAILED reading node with keys ";
    keys.write(cerr, " ", NULL, "()");
    cerr <<" and parents ";
    listWrite(parents,cerr," ","()");
    cerr <<endl;
  }

  //eat the next , or ;
  c=mlr::getNextChar(is," \n\r\t");
  if(c==',' || c==';') {} else is.putback(c);

  return node;
}

#undef PARSERR

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

void Graph::writeHtml(std::ostream& os, std::istream& is) {
  char c;
  long int g=getParseInfo(NULL).beg;
  is.seekg(g);
#define GO { is.get(c); if(c=='\n') os <<"<br>" <<endl; else os <<c; g++; }
  for(Node *n:list()){
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
  while(g<getParseInfo(NULL).end)GO
#undef GO
}

void Graph::writeDot(std::ostream& os, bool withoutHeader, bool defaultEdges, int nodesOrEdges, int focusIndex) {
  if(!withoutHeader){
    os <<"digraph G{" <<endl;
    os <<"graph [ rankdir=\"LR\", ranksep=0.05 ];" <<endl;
    os <<"node [ fontsize=9, width=.3, height=.3 ];" <<endl;
    os <<"edge [ arrowtail=dot, arrowsize=.5, fontsize=6 ];" <<endl;
    index(true);
  }
  for(Node *n: list()) {
    mlr::String label;
    if(n->keys.N){
      label <<"label=\"";
      bool newline=false;
      for(mlr::String& k:n->keys){
        if(newline) label <<'\n';
        label <<k;
        newline=true;
      }
      label <<"\" ";
    }else if(n->parents.N){
      label <<"label=\"(" <<n->parents(0)->keys.last();
      for(uint i=1;i<n->parents.N;i++) label <<' ' <<n->parents(i)->keys.last();
      label <<")\" ";
    }

    mlr::String shape;
    if(n->keys.contains("box")) shape <<" shape=box"; else shape <<" shape=ellipse";
    if(focusIndex==(int)n->index) shape <<" color=red";
    if(hasRenderingInfo(n)) shape <<' ' <<getRenderingInfo(n).dotstyle;


    if(defaultEdges && n->parents.N==2){ //an edge
      os <<n->parents(0)->index <<" -> " <<n->parents(1)->index <<" [ " <<label <<"];" <<endl;
    }else{
      if(n->isGraph()){
        os <<"subgraph cluster_" <<n->index <<" { " <<label /*<<" rank=same"*/ <<endl;
        n->graph().writeDot(os, true, defaultEdges, +1);
        os <<"}" <<endl;
        n->graph().writeDot(os, true, defaultEdges, -1);
      }else{//normal node
        if(nodesOrEdges>=0){
          os <<n->index <<" [ " <<label <<shape <<" ];" <<endl;
        }
        if(nodesOrEdges<=0){
          for_list(Node, pa, n->parents) {
            if(pa->index<n->index)
              os <<pa->index <<" -> " <<n->index <<" [ ";
            else
              os <<n->index <<" -> " <<pa->index <<" [ ";
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
    if(it->isGraph()) {
      double *order = it->graph().find<double>("dot_order");
      if(!order) { MLR_MSG("doesn't have dot_order attribute"); return; }
      perm(it_COUNT) = (uint)*order;
    }
  }
  permuteInv(perm);
  for_list(Node, it2, list()) it2->index=it2_COUNT;
}

ParseInfo& Graph::getParseInfo(Node* n){
  if(pi.N!=N+1){
    listResizeCopy(pi, N+1);
    pi(0)->node=NULL;
    for(uint i=1;i<pi.N;i++) pi(i)->node=elem(i-1);
  }
  if(!n) return *pi(0);
  return *pi(n->index+1);
}

RenderingInfo& Graph::getRenderingInfo(Node* n){
  if(ri.N!=N+1){
    listResizeCopy(ri, N+1);
    ri(0)->node=NULL;
    for(uint i=1;i<ri.N;i++) ri(i)->node=elem(i-1);
  }
  if(!n) return *ri(0);
  return *ri(n->index+1);

}

bool Graph::checkConsistency() const{
  uint idx=0;
  for(Node *node: *this){
    CHECK_EQ(&node->container, this, "");
    CHECK_EQ(node->index, idx, "");
    for(Node *j: node->parents)  CHECK(j->parentOf.findValue(node) != -1,"");
    for(Node *j: node->parentOf) CHECK(j->parents.findValue(node) != -1,"");
    for(Node *parent: node->parents) if(&parent->container!=this){
      //check that parent is contained in a super-graph of this
      const Graph *parentGraph = this;
      const Node *parentGraphNode;
      while(&parent->container!=parentGraph){
        //we need to descend one more
        parentGraphNode = parentGraph->isNodeOfParentGraph;
        CHECK(parentGraphNode,"there is no more supergraph to find the parent");
        parentGraph = &parentGraphNode->container;
      }
      //check sorting
//      CHECK(parent->index < parentGraphNode->index,"subnode refers to parent that sorts below the subgraph");
    }else{
//      CHECK(parent->index < node->index,"node refers to parent that sorts below the node");
    }
    if(node->isGraph()){
      Graph& G = node->graph();
      CHECK_EQ(G.isNodeOfParentGraph, node, "");
      G.checkConsistency();
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
    if(it->isGraph()){
      Graph& G=it->graph();
      if(subKVG) idx = G.index(true, idx);
      else G.index(false, 0);
    }
  }
  return idx;
}

bool operator==(const Graph& A, const Graph& B){
  if(A.N!=B.N) return false;
  for(uint i=0;i<A.N;i++){
    Node *a = A(i), *b = B(i);
    if(a->index!=b->index) return false;
    if(a->keys!=b->keys) return false;
    if(a->parents.N!=b->parents.N) return false;
    for(uint j=0;j<a->parents.N;j++) if(a->parents(j)->index!=b->parents(j)->index) return false;
    if(a->type!=b->type) return false;
    if(!a->hasEqualValue(b)) return false;
  }
  return true;
}

//===========================================================================

Node_typed<Graph>* newSubGraph(Graph& container, const StringA& keys, const NodeL& parents){
  return new Node_typed<Graph>(container, keys, parents, Graph());
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
GraphEditCallbackL::memMove=true;
RUN_ON_INIT_END(graph)
