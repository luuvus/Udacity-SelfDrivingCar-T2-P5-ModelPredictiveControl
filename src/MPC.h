#ifndef MPC_H
#define MPC_H

#define HAVE_CSTDDEF

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  vector<double> trajectory_x;
  vector<double> trajectory_y;  
};

#endif /* MPC_H */
