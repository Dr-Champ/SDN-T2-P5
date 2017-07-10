#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // number of steps the MPC solves into the future
  int nSteps;

  // maximum steering angle limit (rad)
  double maxSteeringRad;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, 
  		double desiredSpeed);
};

#endif /* MPC_H */
