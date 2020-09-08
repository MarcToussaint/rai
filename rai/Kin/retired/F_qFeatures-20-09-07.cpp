void F_qItself::phi(arr& y, arr& J, const ConfigurationL& Ctuple) {
  Feature::phi(y, J, Ctuple);
  return;

  CHECK_GE(Ctuple.N, order+1, "I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0) {
    phi(y, J, *Ctuple(-1));
    if(!!J) expandJacobian(J, Ctuple, -1);
    return;
  }

  double tau = Ctuple(-1)->frames(0)->tau; // - Ktuple(-2)->frames(0)->time;
  double tau2=tau*tau, tau3=tau2*tau;
  arrA q_bar(k+1), J_bar(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = Ctuple.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  //before reading out, check if, in selectedBodies mode, some of the selected ones where switched
  uintA selectedBodies_org = frameIDs;
  if(frameIDs.nd==1) {
    uintA sw = getSwitchedBodies(*Ctuple.elem(-2), *Ctuple.elem(-1));
    for(uint id:sw) frameIDs.removeValue(id, false);
  }
  for(uint i=0; i<=k; i++) {
    if(!!J && isSparseMatrix(J)) J_bar(i).sparse();
    phi(q_bar(i), J_bar(i), *Ctuple(offset+i));
  }
  frameIDs = selectedBodies_org;

  bool handleSwitches=false;
  uint qN=q_bar(0).N;
  for(uint i=0; i<=k; i++) if(q_bar(i).N!=qN) { handleSwitches=true; break; }
  if(handleSwitches) { //when bodies are selected, switches don't have to be handled
    CHECK(!frameIDs.nd, "doesn't work for this...")
    uint nFrames = Ctuple(offset)->frames.N;
    JointL jointMatchLists(k+1, nFrames); //for each joint of [0], find if the others have it
    jointMatchLists.setZero();
    boolA useIt(nFrames);
    useIt = true;
    for(uint i=0; i<nFrames; i++) {
      rai::Frame* f = Ctuple(offset)->frames(i);
      rai::Joint* j = f->joint;
      if(j) {
        for(uint s=0; s<=k; s++) {
          rai::Joint* jmatch = Ctuple(offset+s)->getJointByFrameNames(j->from()->name, j->frame->name);
          if(jmatch && j->type!=jmatch->type) jmatch=nullptr;
          if(!jmatch) { useIt(i) = false; break; }
          jointMatchLists(s, i) = jmatch;
        }
      } else {
        useIt(i) = false;
      }
    }

    arrA q_bar_mapped(k+1), J_bar_mapped(k+1);
    uint qidx, qdim;
    for(uint i=0; i<nFrames; i++) {
      if(useIt(i)) {
        for(uint s=0; s<=k; s++) {
          qidx = jointMatchLists(s, i)->qIndex;
          qdim = jointMatchLists(s, i)->qDim();
          if(qdim) {
            q_bar_mapped(s).append(q_bar(s)({qidx, qidx+qdim-1}));
            if(!!J) J_bar_mapped(s).append(J_bar(s)({qidx, qidx+qdim-1}));
          }
        }
      }
    }

    q_bar = q_bar_mapped;
    if(!!J) J_bar = J_bar_mapped;
  }

  if(k==1)  y = (q_bar(1)-q_bar(0))/tau; //penalize velocity
  if(k==2)  y = (q_bar(2)-2.*q_bar(1)+q_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (q_bar(3)-3.*q_bar(2)+3.*q_bar(1)-q_bar(0))/tau3; //penalize jerk
  if(!!J) {
    uintA qidx = getKtupleDim(Ctuple);
    qidx.prepend(0);
    if(!isSparseMatrix(J)) {
      J = zeros(y.N, qidx.last());
      if(k==1) { J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau; }
      if(k==2) { J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0), 0, qidx(offset+0));  J/=tau2; }
      if(k==3) { J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
    } else {
      J.sparse().resize(y.N, qidx.last(), 0);
      if(k==1) { J_bar(0) *= -1.;  J+=J_bar(0);  J+=J_bar(1);  J/=tau; }
      if(k==2) { J_bar(1) *= -2.;  J+=J_bar(0);  J+=J_bar(1);  J+=J_bar(2);  J/=tau2; }
      if(k==3) { NIY }
    }

    arr Jtau;  Ctuple(-1)->jacobian_tau(Jtau, Ctuple(-1)->frames(0));  expandJacobian(Jtau, Ctuple, -1);
//    arr Jtau2;  Ktuple(-2)->jacobianTime(Jtau2, Ktuple(-2)->frames(0));  expandJacobian(Jtau2, Ktuple, -2);
//    arr Jtau = Jtau1 - Jtau2;
    if(k==1) J += (-1./tau)*y*Jtau;
  }
}

uint F_qItself::dim_phi(const ConfigurationL& Ctuple) {
  if(order==0) return dim_phi(*Ctuple.last());
  else {
    if(dimPhi.find(Ctuple.last()) == dimPhi.end()) {
      arr y;
      phi(y, NoArr, Ctuple);
      dimPhi[Ctuple.last()] = y.N;
      return y.N;
    } else {
      return dimPhi[Ctuple.last()];
    }
  }
  return 0;
}

void F_qItself::signature(intA& S, const rai::Configuration& C) {
  CHECK(frameIDs.N, "");
  S.clear();
  for(uint i=0; i<frameIDs.d0; i++) {
    rai::Joint* j=0;
    if(frameIDs.nd==1) {
      rai::Frame* f = C.frames.elem(frameIDs.elem(i));
      j = f->joint;
      CHECK(j, "selected frame " <<frameIDs.elem(i) <<" ('" <<f->name <<"') is not a joint");
    } else {
      rai::Frame* a = C.frames.elem(frameIDs(i, 0));
      rai::Frame* b = C.frames.elem(frameIDs(i, 1));
      if(a->parent==b) j=a->joint;
      else if(b->parent==a) j=b->joint;
      else HALT("a and b are not linked");
      CHECK(j, "");
    }
    for(uint k=0; k<j->qDim(); k++) S.append(j->qIndex+k);
  }
}
