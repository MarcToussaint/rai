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

struct ParseInfo{
  Node *node;
  istream::pos_type beg,end;
  istream::pos_type err_beg, err_end;
  istream::pos_type keys_beg, keys_end;
  istream::pos_type parents_beg, parents_end;
  istream::pos_type value_beg, value_end;
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
  CHECK(&container!=&NoGraph, "don't do that anymore!");
  index=container.N;
  container.NodeL::append(this);
}

Node::Node(Graph& _container, const StringA& _keys, const NodeL& _parents)
  : container(_container), keys(_keys), parents(_parents){
  CHECK(&container!=&NoGraph, "don't do that anymore!");
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
  if(!hasValue()) return;
  if(getValueType()==typeid(Graph*)) {
    os <<" {";
    V<Graph*>()->write(os, " ");
    os <<" }";
  } else if(getValueType()==typeid(NodeL)) {
    os <<"=(";
    for(Node *it: (*getValue<NodeL>())) os <<' ' <<it->keys.last();
    os <<" )";
  } else if(getValueType()==typeid(mlr::String)) {
    os <<"=\"" <<*getValue<mlr::String>() <<'"';
  } else if(getValueType()==typeid(mlr::FileToken)) {
    os <<"='" <<getValue<mlr::FileToken>()->name <<'\'';
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

//Graph Node::ParentOf(){
//  Graph G;
//  G.isReferringToNodesOf = &container;
//  G.NodeL::operator=(parentOf);
//  return G;
//}

Nod::Nod(const char* key){
  n = new Node_typed<bool>(G, true);
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

//Graph::Graph(Graph& container, const StringA& keys, const NodeL& parents) : isNodeOfParentGraph(NULL){
//  Node *n = new Node_typed<Graph*>(container, keys, parents, this);
//  CHECK(isNodeOfParentGraph==n,"");
//}


Graph::~Graph() {
  clear();
  if(isNodeOfParentGraph){
    Node_typed<Graph*>* it=dynamic_cast<Node_typed<Graph*>*>(isNodeOfParentGraph);
    CHECK(it,"");
    it->value=NULL;
//    it->ownsValue=false;
  }
}

void Graph::clear() {
  while(N) delete last();
}

Node *Graph::append(const Nod& ni){
  Node *clone = ni.n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
  for(const mlr::String& s:ni.parents){
    Node *p = getNode(s);
    CHECK(p,"parent " <<p <<" of " <<*clone <<" does not exist!");
    clone->parents.append(p);
    p->parentOf.append(clone);
  }
  return clone;
}

Node *Graph::append(const uintA& parentIdxs) {
  NodeL parents(parentIdxs.N);
  for(uint i=0;i<parentIdxs.N; i++) parents(i) = NodeL::elem(parentIdxs(i));
  return append<int>({STRING(NodeL::N)}, parents, 0);
}

void Graph::appendDict(const std::map<std::string, std::string>& dict){
  for(const std::pair<std::string,std::string>& p:dict){
    Node *it = readNode(STRING('='<<p.second), false, false, mlr::String(p.first));
    if(!it) MLR_MSG("failed to read dict entry <" <<p.first <<',' <<p.second <<'>');
  }
}

Node* Graph::getNode(const char *key) const {
  for(Node *it: (*this)) if(it->matches(key)) return it;
  if(isNodeOfParentGraph) return isNodeOfParentGraph->container.getNode(key);
  return NULL;
}

//Node* Graph::getNode(const char *key1, const char *key2) const {
//  for(Node *it: (*this)) {
//    for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key1) {
//      for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key2)
//        return it;
//    }
//  }
//  return NULL;
//}

Node* Graph::getNode(const StringA &keys) const {
  for(Node *it: (*this)) if(it->matches(keys)) return it;
  if(isNodeOfParentGraph) return isNodeOfParentGraph->container.getNode(keys);
  return NULL;
}

NodeL Graph::getNodes(const StringA &keys) const {
  NodeL ret;
  for(const String& s:keys){ Node *n=getNode(s); CHECK(n,"unknown symbol '"<<s <<"'"); ret.append(n); }
  return ret;

}

NodeL Graph::getNodes(const char* key) const {
  NodeL ret;
  for(Node *it: (*this)) if(it->matches(key)) ret.append(it);
  return ret;
}

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

NodeL Graph::getNodesOfDegree(uint deg) {
  NodeL ret;
  for(Node *it: (*this)) {
    if(it->parents.N==deg) ret.append(it);
  }
  return ret;
}


NodeL Graph::getNodesOfType(const char* key, const std::type_info& type) {
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
  NodeL KVG = getNodesOfType(m->keys(0), m->getValueType());
  //CHECK(KVG.N<=1, "can't merge into multiple nodes yet");
  Node *n=NULL;
  if(KVG.N) n=KVG.elem(0);
  CHECK(n!=m,"how is this possible?");
  if(n){
    CHECK(m->getValueType()==n->getValueType(), "can't merge nodes of different types!");
    if(n->getValueType()==typeid(Graph*)){ //merge the KVGs
      n->V<Graph*>()->merge(*m->V<Graph*>());
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

void Graph::xx_graph_copy(const Graph& G, bool appendInsteadOfClear){
  DEBUG(G.checkConsistency());

  //-- first delete existing nodes
  if(!appendInsteadOfClear) clear();
  uint indexOffset=N;
  NodeL newNodes;

  //-- if either is a subgraph, ensure they're a subgraph of the same -- over restrictive!!
//  if(isNodeOfParentGraph || G.isNodeOfParentGraph){
//    CHECK(&isNodeOfParentGraph->container==&G.isNodeOfParentGraph->container,"is already subgraph of another container!");
//  }
//  if(becomeSubgraphOfContainer){ //CHECK that this is also a subgraph of the same container..
//    if(!isNodeOfParentGraph){
//      HALT("don't do that");
//      Node *Gnode = G.isNodeOfParentGraph;
//      if(Gnode)
//        new Node_typed<Graph*>(*becomeSubgraphOfContainer, Gnode->keys, Gnode->parents, this);
//      else
//        new Node_typed<Graph*>(*becomeSubgraphOfContainer, {}, {}, this);
//    }else{
//      CHECK(&isNodeOfParentGraph->container==becomeSubgraphOfContainer,"is already subgraph of another container!");
//    }
//  }

  //-- first, just clone nodes with their values -- 'parents' still point to the origin nodes
  for(Node *n:G){
    Node *newn=NULL;
    if(n->getValueType()==typeid(Graph*) && n->V<Graph*>()!=NULL){
      // why we can't copy the subgraph yet:
      // copying the subgraph would require to fully rewire the subgraph (code below)
      // but if the subgraph refers to parents of this graph that are not create yet, requiring will fail
      // therefore we just insert an empty graph here; we then copy the subgraph once all nodes are created
      newn = newSubGraph(*this, n->keys, n->parents);
    }else{
      newn = n->newClone(*this); //this appends sequentially clones of all nodes to 'this'
    }
    newNodes.append(newn);
  }

  //-- the new nodes are not parent of anybody yet
  for(Node *n:newNodes) CHECK(n->parentOf.N==0,"");

  //-- now copy subgraphs
  for(Node *n:newNodes) if(n->getValueType()==typeid(Graph*) && n->V<Graph*>()!=NULL){
    n->graph().isNodeOfParentGraph = n;
    n->graph().xx_graph_copy(G.elem(n->index-indexOffset)->graph()); //you can only call the operator= AFTER assigning isNodeOfParentGraph
  }

  //-- now rewire parental links
  for(Node *n:newNodes){
    for(uint i=0;i<n->parents.N;i++){
      Node *p=n->parents(i); //the parent in the origin graph
      if(&p->container==&G){ //parent is directly in G, no need for complicated search
        p->parentOf.removeValue(n);   //original parent it not parent of copy
        p = newNodes.elem(p->index);  //the true parent in the new graph
      }else{
        const Graph *newg=this, *oldg=&G;
        while(&p->container!=oldg){  //find the container while iterating backward also in the newG
          CHECK(oldg->isNodeOfParentGraph,"");
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
    char c=mlr::peerNextChar(is, " \n\r\t,");
    if(!is.good() || c=='}') { is.clear(); break; }
    Node *it = readNode(is, false, parseInfo);
    if(!it) break;
    if(it->keys.N==1 && it->keys.last()=="Include"){
      read(it->getValue<mlr::FileToken>()->getIs(true));
      delete it;
    }else
    if(it->keys.N==1 && it->keys.last()=="ChDir"){
      it->getValue<mlr::FileToken>()->changeDir();
    }
  }
  if(parseInfo) getParseInfo(NULL).end=is.tellg();

  //-- merge all Merge keys
  NodeL merges = getNodes("Merge");
  for(Node *m:merges){
    m->keys.remove(0);
    merge(m);
  }

  //-- delete all ChDir nodes in reverse order
  for(uint i=N;i--;){
    Node *it=elem(i);
    if(it->keys.N==1 && it->keys(0)=="ChDir"){
      it->getValue<mlr::FileToken>()->unchangeDir();
      delete it;
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
  cerr <<"<<<\"  ]]" <<endl; is.clear(); }

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

  if(verbose) { cout <<" keys:" <<keys <<flush; }

  //-- read parents
  char c=mlr::getNextChar(is," \t"); //don't skip new lines
  if(c=='(') {
    pinfo.parents_beg=is.tellg();
    for(uint j=0;; j++) {
      if(!str.read(is, " \t\n\r,", " \t\n\r,)", false)) break;
      Node *e=this->getNode(str);
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
          node->V<mlr::FileToken>().getIs();  //creates the ifstream and might throw an error
        } catch(...){
          delete node;
          PARSERR("file which does not exist -> converting to string!", pinfo);
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
          PARSERR("could not parse value of type '" <<str <<"' -- no such type has been registered; ignoring: '"<<substr<<"'", pinfo);
        } else {
          node->keys = keys;
          node->parents = parents;
        }
        mlr::parse(is, ">");
      } break;
      case '{': { // sub graph
        Node_typed<Graph*> *subgraph = newSubGraph(*this, keys, parents);
	subgraph->value->read(is);
        mlr::parse(is, "}");
        node = subgraph;
      } break;
//      case '(': { // referring Graph
//        Graph *refs = new Graph;
//        refs->isReferringToNodesOf = this;
//        for(uint j=0;; j++) {
//          str.read(is, " , ", " , )", false);
//          if(!str.N) break;
//          Node *e=this->getNode(str);
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

  if(parseInfo && node){
    pinfo.node = node;
    node->container.getParseInfo(node) = pinfo;
  }

  if(verbose) {
    if(node) { cout <<" value:"; node->writeValue(cout); cout <<" FULL:"; node->write(cout); cout <<endl; }
    else { cout <<"FAILED" <<endl; }
  }

  if(!node){
    cout <<"FAILED reading node with keys ";
    keys.write(cout, " ", NULL, "()");
    cout <<" and parents ";
    listWrite(parents,cout," ","()");
    cout <<endl;
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
  for(Node *it: list()) {
    mlr::String label, shape("shape=ellipse");
    if(it->keys.N){
      label <<"label=\"";
      bool newline=false;
      for(mlr::String& k:it->keys){
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

    if(focusIndex==(int)it->index) shape <<" color=red";

    if(defaultEdges && it->parents.N==2){ //an edge
      os <<it->parents(0)->index <<" -> " <<it->parents(1)->index <<" [ " <<label <<"];" <<endl;
    }else{
      if(it->getValueType()==typeid(Graph*)){
        os <<"subgraph cluster_" <<it->index <<" { " <<label /*<<" rank=same"*/ <<endl;
        it->V<Graph*>()->writeDot(os, true, defaultEdges, +1);
        os <<"}" <<endl;
        it->V<Graph*>()->writeDot(os, true, defaultEdges, -1);
      }else{//normal node
        if(nodesOrEdges>=0){
          os <<it->index <<" [ " <<label <<shape <<" ];" <<endl;
        }
        if(nodesOrEdges<=0){
          for_list(Node, pa, it->parents) {
            if(pa->index<it->index)
              os <<pa->index <<" -> " <<it->index <<" [ ";
            else
              os <<it->index <<" -> " <<pa->index <<" [ ";
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
    if(it->getValueType()==typeid(Graph*)) {
      double *order = it->V<Graph*>()->getValue<double>("dot_order");
      if(!order) { MLR_MSG("doesn't have dot_order attribute"); return; }
      perm(it_COUNT) = (uint)*order;
    }
  }
  permuteInv(perm);
  for_list(Node, it2, list()) it2->index=it2_COUNT;
}

ParseInfo& Graph::getParseInfo(Node* it){
  if(pi.N!=N+1){
    listResizeCopy(pi, N+1);
    pi(0)->node=NULL;
    for(uint i=1;i<pi.N;i++) pi(i)->node=elem(i-1);
  }
  if(!it) return *pi(0);
  return *pi(it->index+1);
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
      CHECK(parent->index < node->index,"node refers to parent that sorts below the node");
    }
    if(node->getValueType()==typeid(Graph*) && node->V<Graph*>()){
      Graph& G = node->graph();
      CHECK(G.isNodeOfParentGraph==node,"");
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
    if(it->getValueType()==typeid(Graph*) && it->V<Graph*>()){
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
    if(a->getValueType()!=b->getValueType()) return false;
    if(a->hasValue()!=b->hasValue()) return false;
    if(a->hasValue() && !a->hasEqualValue(b)) return false;
  }
  return true;
}

//===========================================================================

Node_typed<Graph*>* newSubGraph(Graph& container, const StringA& keys, const NodeL& parents){
  return new Node_typed<Graph*>(container, keys, parents, new Graph());
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
