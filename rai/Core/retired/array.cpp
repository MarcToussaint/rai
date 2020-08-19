/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_EXPRESSIONS
void assign(arr& x) {
  CHECK(x.ex, "self-assignment only if it is an expression");
  rai::Ex* e=x.ex;
  x.init();
  x.ex=e;
  assign(x, x);
  delete x.ex;
  x.ex=0;
}

void assign(arr& x, const arr& a) {
  if(!a.ex) { x=a; return; }
  rai::Ex& e=*a.ex;
  if(e.op==rai::UNI) {
    arr* A=(arr*)e.A;
    if(A->ex) assign(*A);
    if(!e.trans && e.mul==1 && e.add==0) { x=*A; return; }
    if(!e.trans && e.mul==1) { scalarPlus(x, *A, *((double*)&e.add)); return; }
    if(!e.trans && e.add==0) { scalarMultiplication(x, *A, *((double*)&e.mul)); return; }
    if(e.mul==1 && e.add==0) { transpose(x, *A); return; }
    HALT("");
  } else {
    arr* A=(arr*)e.A, *B=(arr*)e.B;
    if(A->ex) assign(*A);
    if(B->ex) assign(*B);
    //bool at, bt;
    //double ac, bc, ap, bp;
    switch(e.op) {
      case rai::PROD:
        if(!A->ex && !B->ex) { innerProduct(x, *A, *B); return; }
        HALT("prod");
        break;
      case rai::MUL:
        if(!A->ex && !B->ex) { mult(x, *A, *B); return; }
        HALT("mult");
        break;
      case rai::Div:
        if(!A->ex && !B->ex) { div(x, *A, *B); return; }
        HALT("mult");
        break;
      case rai::OUT:
        if(!A->ex && !B->ex) { outerProduct(x, *A, *B); return; }
        HALT("out");
        break;
      case rai::PLUS:
        if(!A->ex && !B->ex) { plus(x, *A, *B); return; }
        //if(A->ex){ ap=A->ex->add; ac=A->ex->mul; at=A->ex->trans; A=(arr*)A->ex->A; }else{ ap=0; ac=1; at=false; }
        //if(B->ex){ bp=B->ex->add; bc=B->ex->mul; bt=B->ex->trans; B=(arr*)B->ex->A; }else{ bp=0; bc=1; bt=false; }
        //if(!at && !bt && !ap && !bp){ plus(x, ac, *A, bc, *B); return; }
        //if(!at && !bt && !B){ scalarPlus(x, *A, bc); return; }
        HALT("plus");
        break;
      case rai::MINUS:
        if(!A->ex && !B->ex) { minus(x, *A, *B); return; }
        //if(A->ex){ ap=A->ex->add; ac=A->ex->mul; at=A->ex->trans; A=(arr*)A->ex->A; }else{ ap=0; ac=1; at=false; }
        //if(B->ex){ bp=B->ex->add; bc=B->ex->mul; bt=B->ex->trans; B=(arr*)B->ex->A; }else{ bp=0; bc=1; bt=false; }
        //if(!at && !bt && !ap && !bp){ plus(x, ac, *A, -bc, *B); return; }
        //if(!at && !bt && !B){ scalarPlus(x, *A, bc); return; }
        HALT("minus");
        break;
      case rai::UNI:
        HALT("shouldn't be here!");
        break;
    }
    HALT("yet undefined expression");
  }
}
#endif
