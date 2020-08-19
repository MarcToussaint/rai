/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

void KOMO::costReport(bool gnuplt) {
  cout <<"*** KOMO -- CostReport" <<endl;

  HALT("deprecated")
  arr& phi = phiMatrix.scalar();
  ObjectiveTypeA& tt = ttMatrix.scalar();

  arr plotData=zeros(T, tasks.N);

  //-- collect all task costs and constraints
  double a;
  arr taskC=zeros(tasks.N);
  arr taskG=zeros(tasks.N);
  uint M=0;
  for(uint t=0; t<T; t++) {
    for(uint i=0; i<tasks.N; i++) {
      Task* c = tasks(i);
      if(!c->isActive(t)) continue;
      uint d=c->feat.dim_phi(configurations({t, t+k_order}), t);

      if(tt.N) for(uint i=0; i<d; i++) CHECK_EQ(tt(M+i), c->type, "");

      if(d) {
        if(c->type==OT_sos) {
          taskC(i) += a = sumOfSqr(phi.sub(M, M+d-1));
          plotData(t, i) = a;
        }
        if(c->type==OT_ineq) {
          double gpos=0., gall=0.;
          for(uint j=0; j<d; j++) {
            double g=phi(M+j);
            if(g>0.) gpos+=g;
            gall += g;
          }
          taskG(i) += gpos;
          plotData(t, i) = gpos; //gall;
        }
        if(c->type==OT_eq) {
          double gpos=0., gall=0.;
          for(uint j=0; j<d; j++) {
            double h=phi(M+j);
            gpos+=fabs(h);
            gall += h;
          }
          taskG(i) += gpos;
          plotData(t, i) = gpos; //all;
        }
      }

      M += d;
    }
  }
  CHECK_EQ(M, phi.N, "");

  //-- generate output
  cout <<" * task costs:" <<endl;
  double totalC=0., totalG=0.;
  for(uint i=0; i<tasks.N; i++) {
    Task* c = tasks(i);
    cout <<"\t '" <<c->name <<"' order=" <<c->feat.order <<" type=" <<c->type;
    cout <<" \tcosts=" <<taskC(i) <<" \tconstraints=" <<taskG(i) <<endl;
    totalC += taskC(i);
    totalG += taskG(i);
  }

  cout <<"\t total task        = " <<totalC <<endl;
  cout <<"\t total constraints = " <<totalG <<endl;

  //-- write a nice gnuplot file
  ofstream fil("z.costReport");
  //first line: legend
  for(auto c:tasks) {
//    uint d=c->feat.dim_phi(world);
    fil <<c->name <<' '; // <<'[' <<d <<"] ";
  }
  for(auto c:tasks) {
    if(c->type==OT_ineq && dualSolution.N) {
      fil <<c->name <<"_dual ";
    }
  }
  fil <<endl;
  //rest: just the matrix?
  if(!dualSolution.N) {
    plotData.write(fil, nullptr, nullptr, "  ");
  } else {
    dualSolution.reshape(T, dualSolution.N/(T));
    catCol(plotData, dualSolution).write(fil, nullptr, nullptr, "  ");
  }
  fil.close();

  ofstream fil2("z.costReport.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'costReport ( plotting sqrt(costs) )'" <<endl;
  fil2 <<"plot 'z.costReport' \\" <<endl;
  for(uint i=1; i<=tasks.N; i++) fil2 <<(i>1?"  ,''":"     ") <<" u 0:"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
  if(dualSolution.N) for(uint i=0; i<tasks.N; i++) fil2 <<"  ,'' u 0:"<<1+tasks.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  if(gnuplt) gnuplot("load 'z.costReport.plt'");
}

void KOMO::temporallyAlignKinematicSwitchesInConfiguration(uint t) {
  for(rai::KinematicSwitch* sw:switches) if(sw->timeOfApplication<=t) {
      sw->temporallyAlign(*configurations(t+k_order-1), *configurations(t+k_order), sw->timeOfApplication==t);
    }
}

