/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "../Core/array.h"

#include <vector>

struct Hungarian {
  arr costs, starred;
  Hungarian(const arr& cost_matrix);
  ~Hungarian();

  uint getMatch_row(uint i) { return starred[i].argmax(); }

 private:
  arr primed;
  uint dim;
  arr covered_rows;
  arr covered_cols;

  void minimize();
  void starZeros();
  void coverColumns();
  void prime();
  void makePath();
  void modifyCost();

  std::vector<uint> path_row;
  std::vector<uint> path_col;
};
