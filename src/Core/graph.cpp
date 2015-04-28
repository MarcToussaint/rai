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

ItemL& NoItemL=*((ItemL*)NULL);
Graph& NoGraph=*((Graph*)NULL);

//===========================================================================

struct ParseInfo{
  Item *it;
  istream::pos_type beg,end;
  istream::pos_type err_beg, err_end;
  enum Error{ good=0, unknownParent };
};

//===========================================================================
//
//  Item methods
//

Item::Item(Graph& _container):container(_container){
  if(&container!=&NoGraph){
    index=container.N;
    container.ItemL::append(this);
  }else{
    index=(uint)(-1);
  }
}

Item::Item(Graph& _container, const ItemL& _parents)
  : container(_container), parents(_parents){
  index=container.N;
  container.ItemL::append(this);
  for(Item *i: parents){
    CHECK(i,"you gave me a NULL parent");
    i->parentOf.append(this);
  }
}

Item::~Item() {
  for(Item *i: parents) i->parentOf.removeValue(this);
  for(Item *i: parentOf) i->parents.removeValue(this);
  container.removeValue(this);
  container.index();
}

bool Item::matches(const char *key){
  for(const MT::String& k:keys) if(k==key) return true;
  return false;
}

bool Item::matches(const StringA &query_keys) {
  for(const MT::String& k:query_keys) {
    if(!matches(k)) return false;
  }
  return true;
}

void Item::write(std::ostream& os) const {
  //-- write keys
  keys.write(os, " ", "", "\0\0");
  
  //-- write parents
  if(parents.N) {
    //    if(keys.N) os <<' ';
    os <<'(';
    for_list(Item, it, parents) {
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
  } else if(getValueType()==typeid(ItemL)) {
    os <<"=(";
    for(Item *it: (*getValue<ItemL>())) os <<' ' <<it->keys.last();
    os <<" )";
  } else if(getValueType()==typeid(MT::String)) {
    os <<"=\"" <<*getValue<MT::String>() <<'"';
  } else if(getValueType()==typeid(MT::FileToken)) {
    os <<"='" <<getValue<MT::FileToken>()->name <<'\'';
  } else if(getValueType()==typeid(arr)) {
    os <<'=' <<*getValue<arr>();
  } else if(getValueType()==typeid(double)) {
    os <<'=' <<*getValue<double>();
  } else if(getValueType()==typeid(bool)) {
    if(*getValue<bool>()) os<<','; else os <<'!';
  } else {
    Item *it = reg_findType(getValueType().name());
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

Graph Item::ParentOf(){
  Graph G;
  G.isReferringToItemsOf = &container;
  G.ItemL::operator=(parentOf);
  return G;
}

Item *readItem(Graph& containingKvg, std::istream& is, bool verbose, bool parseInfo, MT::String prefixedKey=MT::String()) {
  MT::String str;
  StringA keys;
  ItemL parents;
  Item *item=NULL;

  if(verbose) { cout <<"\nITEM (line="<<MT::lineCount <<")"; }

#define PARSERR(x) { cerr <<"[[error in parsing Graph file (line=" <<MT::lineCount <<"):\n"\
                          <<"  item keys=" <<keys <<"\n  error=" <<x <<"]]"; is.clear(); }
  
  //-- read keys
  if(!prefixedKey.N){
    MT::skip(is," \t\n\r");
    for(;;) {
      if(!str.read(is, " \t", " \t\n\r,;([{}=", false)) break;
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
      Item *e=containingKvg.getItem(str);
      if(e) { //sucessfully found
        parents.append(e);
      } else { //this element is not known!!
        int rel=0;
        str >>rel;
        if(rel<0 && (int)containingKvg.N+rel>=0){
          e=containingKvg(containingKvg.N+rel);
          parents.append(e);
        }else{
          PARSERR("unknown " <<j <<". parent '" <<str <<"'");
          MT::skip(is, NULL, ")", false);
        }
      }
    }
    MT::parse(is, ")");
    c=MT::getNextChar(is," \t");
  }
  
  if(verbose) { cout <<" parents:"; if(!parents.N) cout <<"none"; else listWrite(parents,cout," ","()"); cout <<flush; }
  
  //-- read value
  if(c=='=' || c=='{' || c=='[' || c=='<' || c=='!' || c=='\'' || c=='"') {
    if(c=='=') c=MT::getNextChar(is," \t");
    if((c>='a' && c<='z') || (c>='A' && c<='Z')) { //MT::String or boolean
      is.putback(c);
      str.read(is, "", " \n\r\t,;}", false);
      if(str=="true") item = new Item_typed<bool>(containingKvg, keys, parents, new bool(true), true);
      else if(str=="false") item = new Item_typed<bool>(containingKvg, keys, parents, new bool(false), true);
      else item = new Item_typed<MT::String>(containingKvg, keys, parents, new MT::String(str), true);
    } else if(MT::contains("-.0123456789", c)) {  //single double
      is.putback(c);
      double d;
      try { is >>d; } catch(...) PARSERR("can't parse double");
      item = new Item_typed<double>(containingKvg, keys, parents, new double(d), true);
    } else switch(c) {
      case '!': { //boolean false
        item = new Item_typed<bool>(containingKvg, keys, parents, new bool(false), true);
      } break;
      case '\'': { //MT::FileToken
        str.read(is, "", "\'", true);
        MT::FileToken *f = new MT::FileToken(str, false);
        try{
          f->getIs(); //creates the ifstream and might throw an error
          item = new Item_typed<MT::FileToken>(containingKvg, keys, parents, f, true);
        } catch(...){
          PARSERR("kvg indicates file which does not exist -> converting to string!");
          item = new Item_typed<MT::String>(containingKvg, keys, parents, new MT::String(str), true);
          delete f;
        }
      } break;
      case '\"': { //MT::String
        str.read(is, "", "\"", true);
        item = new Item_typed<MT::String>(containingKvg, keys, parents, new MT::String(str), true);
      } break;
      case '[': { //arr
        is.putback(c);
        arr reals;
        is >>reals;
        item = new Item_typed<arr>(containingKvg, keys, parents, new arr(reals), true);
      } break;
      case '<': { //any type parser
        str.read(is, " \t", " \t\n\r()`-=~!@#$%^&*()+[]{};'\\:|,./<>?", false);
        //      str.read(is, " \t", " \t\n\r()`1234567890-=~!@#$%^&*()_+[]{};'\\:|,./<>?", false);
        item = readTypeIntoItem(containingKvg, str, is);
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
        item = new Item_typed<Graph>(containingKvg, keys, parents, subList, true);
        subList->isItemOfParentKvg = item;
        subList->read(is);
        MT::parse(is, "}");
      } break;
      case '(': { // referring Graph
        Graph *refs = new Graph;
        refs->isReferringToItemsOf = &containingKvg;
        for(uint j=0;; j++) {
          str.read(is, " , ", " , )", false);
          if(!str.N) break;
          Item *e=containingKvg.getItem(str);
          if(e) { //sucessfully found
            refs->ItemL::append(e);
          } else { //this element is not known!!
            HALT("line:" <<MT::lineCount <<" reading item '" <<keys <<"': unknown "
                 <<j <<"th linked element '" <<str <<"'"); //DON'T DO THIS YET
          }
        }
        MT::parse(is, ")");
        item = new Item_typed<Graph>(containingKvg, keys, parents, refs, true);
        refs->isItemOfParentKvg = item;
      } break;
      default: { //error
        is.putback(c);
        PARSERR("unknown value indicator '" <<c <<"'");
        return NULL;
      }
    }
  } else { //no '=' or '{' -> boolean
    is.putback(c);
    item = new Item_typed<bool>(containingKvg, keys, parents, new bool(true), true);
  }

#undef PARSERR

  if(verbose) {
    if(item) { cout <<" value:"; item->writeValue(cout); cout <<" FULL:"; item->write(cout); cout <<endl; }
    else { cout <<"FAILED" <<endl; }
  }
  
  if(item){
    //    for(Item *it:item->parents) it->parentOf.append(item);
    //    containingKvg.appendItem(item);
  }else {
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


ItemInitializer::ItemInitializer(const char* key){
  it = new Item_typed<bool>(NoGraph, NULL, false);
  it->keys.append(STRING(key));
}


//===========================================================================
//
//  Graph methods
//

struct sKeyValueGraph {
  //  std::map<std::string, Item*> keyMap;
};

Graph::Graph():s(NULL), isReferringToItemsOf(NULL), isItemOfParentKvg(NULL) {
  ItemL::memMove=true;
}

Graph::Graph(const char* filename):s(NULL), isReferringToItemsOf(NULL), isItemOfParentKvg(NULL) {
  ItemL::memMove=true;
  FILE(filename) >>*this;
}

Graph::Graph(const std::map<std::string, std::string>& dict):s(NULL), isReferringToItemsOf(NULL), isItemOfParentKvg(NULL) {
  ItemL::memMove=true;
  appendDict(dict);
}

Graph::Graph(std::initializer_list<ItemInitializer> list) {
  ItemL::memMove=true;
  for(const ItemInitializer& ic:list){
    Item *clone = ic.it->newClone(*this); //this appends sequentially clones of all items to 'this'
    for(const MT::String& s:ic.parents){
      Item *p = getItem(s);
      CHECK(p,"parent " <<p <<" of " <<*clone <<" does not exist!");
      clone->parents.append(p);
      p->parentOf.append(clone);
    }
  }
}

Graph::Graph(const Graph& G):s(NULL), isReferringToItemsOf(NULL), isItemOfParentKvg(NULL) {
  ItemL::memMove=true;
  *this = G;
}

//Graph::Graph(Item *itemOfParentKvg):s(NULL), isReferringToItemsOf(NULL), isItemOfParentKvg(itemOfParentKvg) {
//  ItemL::memMove=true;
//}

Graph::~Graph() {
}

void Graph::clear() {
  if(!isReferringToItemsOf){
    checkConsistency();
    while(N) delete last();
    checkConsistency();
  }else{
    ItemL::clear();
  }
}

Item *Graph::append(const uintA& parentIdxs) {
  ItemL parents(parentIdxs.N);
  for(uint i=0;i<parentIdxs.N; i++) parents(i) = ItemL::elem(parentIdxs(i));
  return append<int>({STRING(ItemL::N)}, parents, NULL, false);
}

void Graph::appendDict(const std::map<std::string, std::string>& dict){
  for(const std::pair<std::string,std::string>& p:dict){
    Item *it = readItem(*this, STRING('='<<p.second), false, false, MT::String(p.first));
    if(!it) MT_MSG("failed to read dict entry <" <<p.first <<',' <<p.second <<'>');
  }
}

Item* Graph::getItem(const char *key) const {
  for(Item *it: (*this)) if(it->matches(key)) return it;
  //    for(const MT::String& k:it->keys) if(k==key) return it;
  if(isItemOfParentKvg) return isItemOfParentKvg->container.getItem(key);
  return NULL;
}

Item* Graph::getItem(const char *key1, const char *key2) {
  for(Item *it: (*this)) {
    for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key1) {
      for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key2)
        return it;
    }
  }
  return NULL;
}

Item* Graph::getItem(const StringA &keys) {
  //  bool found;
  for(Item *it: (*this)) if(it->matches(keys)) return it;
  if(isItemOfParentKvg) return isItemOfParentKvg->container.getItem(keys);
  //  {
  //    found = true;
  //    for(uint k = 0; k < keys.N && found; k++) {
  //      found = false;
  //      for(const String &key: it->keys) {
  //        if(keys(k) == key) {
  //          found = true;
  //          break;
  //        }
  //      }
  //    }
  //    if(found) return it;
  //  }
  return NULL;
}

Graph Graph::getItems(const char* key) {
  Graph ret;
  ret.isReferringToItemsOf = this;
  for(Item *it: (*this)) if(it->matches(key)) ret.ItemL::append(it);
  //    for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) { ret.ItemL::append(it); break; }
  //  }
  return ret;
}

Item* Graph::getChild(Item *p1, Item *p2) const{
  if(p1->parentOf.N < p2->parentOf.N){
    for(Item *i:p1->parentOf){
      if(p2->parentOf.findValue(i)>0) return i;
    }
  }else{
    for(Item *i:p2->parentOf){
      if(p1->parentOf.findValue(i)>0) return i;
    }
  }
  return NULL;
}

Graph Graph::getItemsOfDegree(uint deg) {
  Graph ret;
  ret.isReferringToItemsOf = this;
  for(Item *it: (*this)) {
    if(it->parents.N==deg) ret.ItemL::append(it);
  }
  return ret;
}


Graph Graph::getTypedItems(const char* key, const std::type_info& type) {
  Graph ret;
  ret.isReferringToItemsOf = this;
  for(Item *it: (*this)) if(it->getValueType()==type) {
    if(!key) ret.ItemL::append(it);
    else for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) {
      ret.ItemL::append(it);
      break;
    }
  }
  return ret;
}

Item* Graph::merge(Item *m){
  Graph KVG = getTypedItems(m->keys(0), m->getValueType());
  //CHECK(KVG.N<=1, "can't merge into multiple items yet");
  Item *it=NULL;
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
      Item *it = m->newClone(*this);
      for(uint i=0;i<it->parents.N;i++){
        it->parents(i) = elem(it->parents(i)->index);
        it->parents(i)->parentOf.append(it);
      }
    }
    return m;
  }
  return NULL;
}

Graph& Graph::operator=(const Graph& G) {
  G.checkConsistency();
  //  G.index();//necessary, after checkConsistency?
  //  { for_list(Item, i, G) i->index=i_COUNT; }

  if(!isReferringToItemsOf){ while(N) delete last(); } // listDelete(*this);
  for(Item *it:G){
    if(it->getValueType()==typeid(Graph)){
      Item *clone = new Item_typed<Graph>(*this, it->keys, it->parents, new Graph(), true);
      clone->parentOf.clear();
      clone->kvg().isItemOfParentKvg=clone;
      clone->kvg().operator=(it->kvg()); //you can only call the operator= AFTER assigning isItemOfParentKvg
    }else{
      Item *clone = it->newClone(*this); //this appends sequentially clones of all items to 'this'
      clone->parentOf.clear();
    }
  }

  //rewire links
  for(Item *it:*this){
    for(uint i=0;i<it->parents.N;i++){
      Item *p=it->parents(i);
      const Graph *newg=this, *oldg=&G;
      while(&p->container!=oldg){  //find the container while iterating backward also in the newG
        CHECK(oldg->isItemOfParentKvg,"");
        newg = &newg->isItemOfParentKvg->container;
        oldg = &oldg->isItemOfParentKvg->container;
      }
      CHECK(p==oldg->elem(p->index),""); //we found the parent in oldg
      p->parentOf.removeValue(it);
      p = newg->elem(p->index); //now assign it to the same in newg
      p->parentOf.append(it);
      it->parents(i)=p;
    }
  }

  this->checkConsistency();
  G.checkConsistency();
  return *this;
}

void Graph::read(std::istream& is, bool parseInfo) {
  if(parseInfo) getParseInfo(NULL).beg=is.tellg();
  for(;;) {
    char c=MT::peerNextChar(is, " \n\r\t,");
    if(!is.good() || c=='}') { is.clear(); break; }
    Item *it = readItem(*this, is, false, parseInfo);
    if(!it) break;
    if(it->keys.N==1 && it->keys(0)=="Include"){
      read(it->getValue<MT::FileToken>()->getIs(true));
      delete it;
    }
  }
  if(parseInfo) getParseInfo(NULL).end=is.tellg();
  //-- merge all Merge keys
  Graph merges = getItems("Merge");
  for(Item *m:merges){
    m->keys.remove(0);
    merge(m);
  }
}

void Graph::write(std::ostream& os, const char *ELEMSEP, const char *delim) const {
  uint i;
  if(delim) os <<delim[0];
  for(i=0; i<N; i++) { if(i) os <<ELEMSEP;  if(elem(i)) elem(i)->write(os); else os <<"<NULL>"; }
  if(delim) os <<delim[1] <<std::flush;
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
  for(Item *it: list()) {
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

    if(defaultEdges && it->parents.N==2 && it->getValueType()==typeid(bool)){ //an edge
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
          for_list(Item, pa, it->parents) {
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
  for_list(Item, it, list()) {
    if(it->getValueType()==typeid(Graph)) {
      double *order = it->getValue<Graph>()->getValue<double>("dot_order");
      if(!order) { MT_MSG("doesn't have dot_order attribute"); return; }
      perm(it_COUNT) = (uint)*order;
    }
  }
  permuteInv(perm);
  for_list(Item, it2, list()) it2->index=it2_COUNT;
}

ParseInfo& Graph::getParseInfo(Item* it){
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
  for(Item *it: *this){
    CHECK_EQ(&it->container, this, "");
    CHECK_EQ(it->index, idx, "");
    for(Item *j: it->parents)  CHECK(j->parentOf.findValue(it) != -1,"");
    for(Item *j: it->parentOf) CHECK(j->parents.findValue(it) != -1,"");
    for(Item *parent: it->parents) if(&parent->container!=this){
      //check that parent is contained in a super-graph of this
      const Graph *parentGraph = this;
      const Item *parentGraphItem;
      while(&parent->container!=parentGraph){
        //wee need to descend one more
        parentGraphItem = parentGraph->isItemOfParentKvg;
        CHECK(parentGraphItem,"there is no more supergraph to find the parent");
        parentGraph = &parentGraphItem->container;
      }
      //check sorting
//      CHECK(parent->index < parentGraphItem->index,"subitem refers to parent that sorts below the subgraph");
    }else{
      CHECK(parent->index < it->index,"item refers to parent that sorts below the item");
    }
    if(it->getValueType()==typeid(Graph)){
      Graph& G = it->kvg();
      CHECK(G.isItemOfParentKvg==it,"");
      if(!G.isReferringToItemsOf) G.checkConsistency();
    }
    idx++;
  }
  return true;
}

uint Graph::index(bool subKVG, uint start){
  uint idx=start;
  for(Item *it: list()){
    it->index=idx;
    idx++;
    if(it->getValueType()==typeid(Graph)){
      Graph& G=it->kvg();
      if(!G.isReferringToItemsOf){
        if(subKVG) idx = G.index(true, idx);
        else G.index(false, 0);
      }
    }
  }
  return idx;
}
