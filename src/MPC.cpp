#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// REVIEW: Set the timestep length and duration
size_t N = 8; // num of timesteps
double dt = 0.12; // time per step (seconds)

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
const double Lf = 2.67; //m

// Set and transform reference velocity into m/s
double ref_v = 90 * 0.44704;

// set index for vector variables
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// Object of MPC cost function and constraints 
// and input for IPOPT optimizer
class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Initialize cost at 0
    fg[0] = 0;
    // Define weights for the cost function and set
    for (unsigned int t = 0; t < N; ++t) {
      // Penalize the cross track error - basic CTE
      fg[0] += 20 * CppAD::pow(vars[cte_start + t], 2);
      // Penalize the psi orientation error - progressive linear by time unit
      fg[0] += 3000 * CppAD::pow(vars[epsi_start + t], 2);
      // Penalize the difference from the reference velocity - progressive linear by time unit
      fg[0] += 1000 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    // Minimize utilization of actuations (control input amplitude)
    for (unsigned int t = 0; t < N - 1; ++t) {
      fg[0] += 5000 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 500 * CppAD::pow(vars[a_start + t], 2);
    }
    // Minimize change rate between two sequencial actuations for smoothening
    for (unsigned int t = 0; t < N - 2; ++t) {
      fg[0] += 500 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); 
      fg[0] += 50 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Set the constraints at initial state time 0
    // Adding 1 in correlation of cost located at index 0 of `fg`
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (unsigned int t = 1; t < N; ++t) {
      // State at time, t+1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // State at time, t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Actuations at time, t
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // Errors at time, t
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));

      // Set up the remaining constraints for time steps [1,N]
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}


/* 
 * The Solve() method minimize a cost function 
 * while satisfying constraints on the state variables.
 * It optimizes a planned trajectory path and actuations
 * using the current state vector and waypoint polyfit coefficients.
 *
 * Returns a vector with steering and throttle from the
 * time steps of the MPC's planned driving path. 
 */
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // REVIEW: Set the number of constraints
  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N * 6 + (N - 1) * 2;  //(6N state elements, 2(N-1) actuations)
  // Set the number of constraints
  size_t n_constraints = N * 6;  //(x, y, v, psi, cte, epsi)

  // Initialize model variables to zero
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  // 6 state variables: x, y, psi, v, cte, epsi)
  // Set the initial state
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // REVIEW: Set lower and upper bounds for variables.
  // Set all non-actuators upper and lower bounds to max negative and positive
  for (unsigned int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19; // init all lower bounds to no limit
    vars_upperbound[i] = 1.0e19;  // init all upper bounds to no limit
  }

  // The steering actuator upper and lower bounds of delta are set 
  for (unsigned int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332; // -25 deg => rad
    vars_upperbound[i] = 0.436332;  // +25 deg => rad
  }

  // Acceleration actuator upper and lower bounds.
  for (unsigned int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper bounds for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Set constraints bounds to current state 
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // The "fg_eval" object holds cost function and constraints.
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
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
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  //cout << "SUCCESS: " << (CppAD::ipopt::solve_result<Dvector>::success) << endl;
  
  // Cost
  double cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // REVIEW: Return actuator values
  std::vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  // Prep vector of MPC planned path x,y coords 
  x_vals.clear();
  y_vals.clear();
  // push back the predicted x,y values into the attributes for visualization
  for (unsigned int i = 1; i < N; ++i) {
    x_vals.push_back(solution.x[x_start + i]);
    y_vals.push_back(solution.x[y_start + i]);
  }

  return result;
}