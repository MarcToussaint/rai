/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "i_Ipopt.h"

#ifdef RAI_IPOPT

#include "../Core/util.h"

#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>

struct Conv_NLP2Ipopt : Ipopt::TNLP {
  shared_ptr<NLP> P;
  arr x, phi_x, J_x;
  intA J_structure;

  //-- buffers to avoid recomputing gradients

  Conv_NLP2Ipopt(const shared_ptr<NLP>& P, const arr& x_init);
  virtual ~Conv_NLP2Ipopt();

  void ensure_eval(uint n, const double* _x, bool new_x);

  virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                            Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);

  virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                               Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

  virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                  bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                  Ipopt::Index m, bool init_lambda,
                                  Ipopt::Number* lambda);

  virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* _x, bool new_x, Ipopt::Number& obj_value);

  virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* _x, bool new_x, Ipopt::Number* grad_f);

  virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* _x, bool new_x, Ipopt::Index m, Ipopt::Number* _g);

  virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* _x, bool new_x,
                          Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol,
                          Ipopt::Number* values);

  virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                      bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                      Ipopt::Index* jCol, Ipopt::Number* values);

  virtual void finalize_solution(Ipopt::SolverReturn status,
                                 Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                 Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
                                 Ipopt::Number obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq);
};

Conv_NLP2Ipopt::Conv_NLP2Ipopt(const shared_ptr<NLP>& P, const arr& x_init) : P(P), x(x_init) {
  if(!P->checkBounds(true)) HALT("IPopt needs strict > bounds");

  ensure_eval(x_init.N, x_init.p, true);
  // x = x_init;
  // P->evaluate(phi_x, J_x, x);
  CHECK(rai::isSparseMatrix(J_x), "");
  J_structure = J_x.sparse().elems;
}

Conv_NLP2Ipopt::~Conv_NLP2Ipopt() {}

void Conv_NLP2Ipopt::ensure_eval(uint n, const double* _x, bool new_x){
  if(new_x) {
    CHECK(_x, "");
    CHECK_EQ(x.N, n, "");
    x.setCarray(_x, n);
    P->evaluate(phi_x, J_x, x);
    if(!rai::isSparseMatrix(J_x)) J_x.sparse();
  }
}

shared_ptr<SolverReturn> IpoptInterface::solve(const arr& x_init) {
  Ipopt::IpoptApplication opt;

  //-- set options
  bool ok=true;
  ok &= opt.Options()->SetStringValue("output_file", "z.ipopt.out");

  ok &= opt.Options()->SetNumericValue("tol", 1e-3);
  ok &= opt.Options()->SetNumericValue("constr_viol_tol", 1e-3);
  ok &= opt.Options()->SetNumericValue("compl_inf_tol", 1e-3);

  ok &= opt.Options()->SetIntegerValue("max_iter", 10000);
  // ok &= opt.Options()->SetStringValue("nlp_scaling_method", "none");

  ok &= opt.Options()->SetStringValue("mu_strategy", "adaptive");
  ok &= opt.Options()->SetNumericValue("mu_init", 1e-3);
   // opt.Options()->SetStringValue("hessian_approximation", "limited-memory");
   // opt.Options()->SetStringValue("linear_solver", "ma27");

   // opt.Options()->SetStringValue("derivative_test", "first-order");
   // opt.Options()->SetNumericValue("derivative_test_perturbation", 1e-8);
  //  opt.Options()->SetNumericValue("derivative_test_tol", 1e-4);
  CHECK(ok, "some option could not be set");

  //-- create template NLP structure and set x_init
  Conv_NLP2Ipopt* conv = new Conv_NLP2Ipopt(P, x_init);
  Ipopt::SmartPtr<Ipopt::TNLP> mynlp(conv);

  //-- initialize IPopt
  Ipopt::ApplicationReturnStatus status;
  status = opt.Initialize();
  CHECK_EQ(status, Ipopt::Solve_Succeeded, "Error during Ipopt initialization!");

  //-- optimize
  status = opt.OptimizeTNLP(mynlp);

  if(status == Ipopt::Solve_Succeeded) {
    printf("\n\n*** The problem solved!\n");
  } else {
    printf("\n\n*** The problem FAILED!\n");
  }

  arr err = P->summarizeErrors(conv->phi_x);
  shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
  ret->ineq = err(OT_ineq);
  ret->eq = err(OT_eq);
  ret->sos = err(OT_sos);
  ret->f = err(OT_f);
  ret->feasible = (ret->ineq<.1) && (ret->eq<.1);

  return ret;
}

bool Conv_NLP2Ipopt::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, Ipopt::TNLP::IndexStyleEnum& index_style) {
  n = P->dimension;

  m=0;
  for(auto t:P->featureTypes) {
    if(t==OT_ineq) m++;
    if(t==OT_eq) m++;
  }
  nnz_jac_g = m*n;
  nnz_h_lag = n*n;
  index_style = TNLP::C_STYLE;

  CHECK(n>0, "");
  CHECK_GE(m, 0, "");

  return true;
}

bool Conv_NLP2Ipopt::get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) {
  CHECK_EQ(n, (int)P->dimension, "");

  for(int i=0; i<n; i++) {
    x_l[i] = P->bounds(0,i);
    x_u[i] = P->bounds(1,i);
  }

  int j=0;
  for(auto t:P->featureTypes) {
    if(t==OT_ineq) { g_l[j]=-1e19; g_u[j]=0.; j++; }
    if(t==OT_eq) { g_l[j]=g_u[j]=0.; j++; }
  }
  CHECK_EQ(j, m, "");

  return true;
}

bool Conv_NLP2Ipopt::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* _x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda) {
  CHECK_EQ(init_x, true, "");
  CHECK_EQ(init_z, false, "");
  CHECK_EQ(init_lambda, false, "");

  CHECK_EQ((int)x.N, n, "");
  for(int i=0; i<n; i++) _x[i] = x.elem(i);

  return true;
}

bool Conv_NLP2Ipopt::eval_f(Ipopt::Index n, const Ipopt::Number* _x, bool new_x, Ipopt::Number& obj_value) {

  double f=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes(i)==OT_f) f += phi_x(i);
    if(P->featureTypes(i)==OT_sos) f += rai::sqr(phi_x(i));
  }

  obj_value = f;

  return true;
}

bool Conv_NLP2Ipopt::eval_grad_f(Ipopt::Index n, const Ipopt::Number* _x, bool new_x, Ipopt::Number* grad_f) {
  ensure_eval(n, _x, new_x);

  arr coeff=zeros(phi_x.N);
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes(i)==OT_f) coeff(i) += 1.;                // direct cost term
    if(P->featureTypes(i)==OT_sos) coeff(i) += 2.* phi_x(i);    // sumOfSqr terms
    //      if(muLB     && featureTypes(i)==OT_ineq                 ) coeff(i) -= (muLB/phi_x(i)); //log barrier, check feasibility
    //      if(mu       && featureTypes(i)==OT_ineq && I_lambda_x(i)) coeff(i) += 2.*mu*phi_x(i);  //g-penalty
    //      if(lambda.N && featureTypes(i)==OT_ineq && lambda(i)>0. ) coeff(i) += lambda(i);       //g-lagrange terms
    //      if(nu       && featureTypes(i)==OT_eq                   ) coeff(i) += 2.*nu*phi_x(i);  //h-penalty
    //      if(lambda.N && featureTypes(i)==OT_eq                   ) coeff(i) += lambda(i);       //h-lagrange terms
  }
  arr g;
  g.referTo(grad_f, n);
  g = comp_At_x(J_x, coeff);

  return true;
}

bool Conv_NLP2Ipopt::eval_g(Ipopt::Index n, const Ipopt::Number* _x, bool new_x, Ipopt::Index m, Ipopt::Number* _g) {
  ensure_eval(n, _x, new_x);

  arr g;
  g.referTo(_g, m);
  int j=0;
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes(i)==OT_ineq || P->featureTypes(i)==OT_eq) { g(j)=phi_x(i); j++; }
  }
  CHECK_EQ(j, m, "");

  return true;
}

bool Conv_NLP2Ipopt::eval_jac_g(Ipopt::Index n, const Ipopt::Number* _x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values) {
  ensure_eval(n, _x, new_x);

  CHECK_EQ(phi_x.N, P->featureTypes.N, ""); //was evaluated
  CHECK(rai::isSparseMatrix(J_x), "");
  CHECK_EQ(J_structure, J_x.sparse().elems, "BUMMER! IPopt requires the sparse structure of J to be always the same!!");

  //construct trivial (sparse linear identity) mapping from all features to selection
  arr M;
  M.sparse().resize(m, J_x.d0, m);
  uint j=0;
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes(i)==OT_ineq || P->featureTypes(i)==OT_eq) {
      M.sparse().entry(j, i, j) = 1.;
      j++;
    }
  }
  CHECK_EQ((int)j, m, "");

  arr J = M * J_x;
  CHECK(rai::isSparseMatrix(J), "");
  CHECK_EQ((int)J.d0, m, "");
  rai::SparseMatrix& J_ = J.sparse();

  if(values) {
    memset(values, 0, sizeof(Ipopt::Number)*nele_jac);
  }
  if(iRow && jCol) {
    memset(iRow, 0, sizeof(Ipopt::Index)*nele_jac);
    memset(jCol, 0, sizeof(Ipopt::Index)*nele_jac);
  }
  for(uint i=0; i<J.N; i++) {
    if(values) {
      values[i] = J.p[i];
    }
    if(iRow && jCol) {
      iRow[i] = J_.elems(i, 0);
      jCol[i] = J_.elems(i, 1);
    }
  }

  return true;

}

bool Conv_NLP2Ipopt::eval_h(Ipopt::Index n, const Ipopt::Number* _x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values) {
  CHECK(!new_x, "");

  CHECK_EQ(phi_x.N, P->featureTypes.N, ""); //was evaluated
  CHECK(rai::isSparseMatrix(J_x), "");
  CHECK_EQ(J_structure, J_x.sparse().elems, "BUMMER! IPopt requires the sparse structure of J to be always the same!!");

  arr coeff=zeros(phi_x.N);
  bool hasFterms=false;
  for(uint i=0; i<phi_x.N; i++) {
    if(P->featureTypes(i)==OT_f) hasFterms = true;
    if(P->featureTypes(i)==OT_sos) coeff(i) += 2.;
  }
  arr tmp = J_x;
  arr sqrtCoeff = sqrt(coeff);
  tmp.sparse().rowWiseMult(sqrtCoeff);
  arr H = comp_At_A(tmp); //Gauss-Newton type!

  if(hasFterms) {
    arr H_x;
    P->getFHessian(H_x, x);
    H_x.sparse();
    H += H_x;
  }

  CHECK(rai::isSparseMatrix(H), "");
  rai::SparseMatrix& H_ = H.sparse();

  if(values) {
    memset(values, 0, sizeof(Ipopt::Number)*nele_hess);
  }
  if(iRow && jCol) {
    memset(iRow, 0, sizeof(Ipopt::Index)*nele_hess);
    memset(jCol, 0, sizeof(Ipopt::Index)*nele_hess);
  }
  for(uint i=0; i<H.N; i++) {
    if(values) {
      values[i] = H.p[i];
    }
    if(iRow && jCol) {
      iRow[i] = H_.elems(i, 0);
      jCol[i] = H_.elems(i, 1);
    }
  }

  return true;
}

void Conv_NLP2Ipopt::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* _x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq) {
  x.setCarray(_x, n);
}

#else

shared_ptr<SolverReturn> IpoptInterface::solve(const arr& x_init) {
  NICO
}

#endif
