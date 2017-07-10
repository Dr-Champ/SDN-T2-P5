#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration. 
size_t N = 25;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Global variables for the indices of the parameters in vars
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  // Desired speed in mps
  double desiredSpeed;

  FG_eval(Eigen::VectorXd coeffs, double desiredSpeed) { 
    this->coeffs = coeffs; 
    this->desiredSpeed = desiredSpeed;
  }

  // Evaluate the desired psi (tangent of the line at x)
  AD<double> computeTangent(Eigen::VectorXd coeffs, AD<double> x) {
    AD<double> result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
      result += i * coeffs[i] * pow(x, i - 1);
    }
    return CppAD::atan(result);
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // Some personal explainations how this thing works
    //
    // - The optimizer optimizes the vars (states, errors, actuator signals)
    //   by starting at some point and going to the optimum point with Gradient
    //   Descend (or similar) algorithm. This is done iteratively. It does not
    //   need to know the exact model how each variables in vars are related.
    //
    // - In each iteration, it passes the vars into this method to determine 
    //   corresponding cost and constraints.
    //   -> cost results from taking all errors into account along with some
    //      other "sensible" tricks to influence actuator signals as we see fit.
    //   -> constraints limit the difference between 
    //       (value at state t) - (value as predicted by the model from state t - 1)
    //
    // - So the optimizer learns from the returned cost and constraints what
    //   directions it should modify the variables in vars to meet the pre-set
    //   lower/upper_constraints and minimize the cost in each iteration.
    // 
    // Note that the "iterations" are for the optimizer to iteratively descend
    // the gradient. It is not to be confused with "time steps" of the solution
    // which is the time steps we will be giving actuator signals to the car
    // and measuring state variables. 

    // The cost value
    fg[0] = 0.0; 

    // The initial constraints at time step 0
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Compute the costs and constraints for all time steps
    for (int i = 0; i < N; i++) {
      // cost component: sum of squared errors
      fg[0] += CppAD::pow(vars[cte_start + i], 2);
      fg[0] += CppAD::pow(vars[epsi_start + i], 2);

      // cost component: maintain constant speed
      fg[0] += CppAD::pow(vars[v_start + i] - desiredSpeed, 2);

      // cost component: minimize the use of actuators
      if (i < N - 1) {
        fg[0] += 70 * CppAD::pow(vars[delta_start + i], 2);
        fg[0] += 20 * CppAD::pow(vars[a_start + i], 2);
      }

      // cost component: minimize (square of) fluctuations in subsequent actuator signals
      if ((i > 0) && (i < N - 1)) {
        fg[0] += 70 * CppAD::pow(vars[delta_start + i] - vars[delta_start + i - 1], 2);
        fg[0] += 100 * CppAD::pow(vars[a_start + i] - vars[a_start + i - 1], 2);
      }

      // Update the constraints based on our model of vehicle dynamics.
      if (i > 0) {
        AD<double> x0 = vars[x_start + i];
        AD<double> y0 = vars[y_start + i];
        AD<double> psi0 = vars[psi_start + i];
        AD<double> v0 = vars[v_start + i];
        AD<double> cte0 = vars[cte_start + i];
        AD<double> epsi0 = vars[epsi_start + i];

        AD<double> x1 = vars[x_start + i - 1];
        AD<double> y1 = vars[y_start + i - 1];
        AD<double> psi1 = vars[psi_start + i - 1];
        AD<double> v1 = vars[v_start + i - 1];
        AD<double> epsi1 = vars[epsi_start + i - 1];
        AD<double> delta1 = vars[delta_start + i - 1];
        AD<double> a1 = vars[a_start + i - 1];
        AD<double> psides1 = computeTangent(coeffs, x1);
        AD<double> f1 = 0.0;
        for (int j = 0; j < coeffs.size(); j++) {
          f1 += coeffs[j] * CppAD::pow(x1, j);
        }

        fg[1 + x_start + i] = x0 - (x1 + (v1 * CppAD::cos(psi1) * dt));
        fg[1 + y_start + i] = y0 - (y1 + (v1 * CppAD::sin(psi1) * dt));
        fg[1 + psi_start + i] = psi0 - (psi1 + (v1 * delta1 / Lf * dt));
        fg[1 + v_start + i] = v0 - (v1 + (a1 * dt));
        fg[1 + cte_start + i] = cte0 - (f1 - y1 + (v1 * CppAD::sin(epsi1) * dt));
        fg[1 + epsi_start + i] = epsi0 - (psi1 - psides1 + (v1 * delta1 / Lf * dt));
      }
    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() { nSteps = N; }
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double desiredSpeed) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = (6 * N) + (2 * (N - 1));
  // TODO: Set the number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -maxSteeringRad and 
  // maxSteeringRad (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -maxSteeringRad;
    vars_upperbound[i] = maxSteeringRad;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[psi_start] = state[2];
  constraints_upperbound[v_start] = state[3];
  constraints_upperbound[cte_start] = state[4];
  constraints_upperbound[epsi_start] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, desiredSpeed);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  //std::cout << "Solve vars: " << vars << std::endl;
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  //std::cout << "Optimization result " << ok << std::endl;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost: " << cost << std::endl;

  // return the actuator signals for the next immediate time step
  //std::cout << "solution: " << solution.x << std::endl;
  vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  // return the predicted trajectory from MPC
  for (int i = 0; i < N; i++) {
    result.push_back(solution.x[x_start + i]);
  }

  for (int i = 0; i < N; i++) {
    result.push_back(solution.x[y_start + i]);
  }

  return result;
}
