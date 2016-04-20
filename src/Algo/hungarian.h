#include <Core/array.h>

#include <vector>

struct Hungarian {
  arr costs, starred;
  Hungarian(const arr& cost_matrix);
  ~Hungarian();

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
