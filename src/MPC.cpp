#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

static const size_t TIMESTEPS = 10; // Number of timesteps
static const double DT = 0.1; // Timestep evaluation frequency

static const size_t NUM_VARS = TIMESTEPS * 6 + (TIMESTEPS - 1) * 2;
static const size_t CONSTRAINTS = TIMESTEPS * 6;

// This value assumes the model presented in the classroom is used. It was obtained by measuring the radius formed by running the vehicle in the simulator around in a circle with a
// constant steering angle and velocity on a flat terrain. Lf was tuned until the the radius formed by the simulating the model presented in the classroom matched the previous
// radius. This is the length from front to CoG that has a similar radius.
static const double LF = 2.67;

// Both the reference cross track and orientation errors are 0.
static const double REF_CTE = 0;
static const double REF_EPSI = 0;
static const double REF_V = 200; // The reference velocity is set to 200 mph.

// Adjustment coefficients for the reference state (non-actuators)
static const int CTE_COST_COEFF = 3500;
static const int EPSI_COST_COEFF = 3500;
static const int V_COST_COEFF = 1;

// Adjustment coefficient for delta (steering actuator)
static const int DELTA_COST_COEFF = 5;

// Adjustment coefficients for 'a' (throttle actuator)
static const int A_LOWER_COST_COEFF = 200;
static const int A_UPPER_COST_COEFF = 5;

// The solver takes all the state variables and actuator variables in a singular vector.
// These constants establish start indexes for each variable to make life easier.
static const size_t X_START_IDX = 0;
static const size_t Y_START_IDX = X_START_IDX + TIMESTEPS;
static const size_t PSI_START_IDX = Y_START_IDX + TIMESTEPS;
static const size_t V_START_IDX = PSI_START_IDX + TIMESTEPS;
static const size_t CTE_START_IDX = V_START_IDX + TIMESTEPS;
static const size_t EPSI_START_IDX = CTE_START_IDX + TIMESTEPS;
static const size_t DELTA_START_IDX = EPSI_START_IDX + TIMESTEPS;
static const size_t A_START_IDX = DELTA_START_IDX + TIMESTEPS - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector &fg, const ADvector &vars) {
    // The cost is stored is the first element of `fg`. Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The cost adjustments for the reference state (non-actuators)
    for (int t = 0; t < TIMESTEPS; t++) {
      fg[0] += CTE_COST_COEFF * CppAD::pow(vars[CTE_START_IDX + t] - REF_CTE, 2);
      fg[0] += EPSI_COST_COEFF * CppAD::pow(vars[EPSI_START_IDX + t] - REF_EPSI, 2);
      fg[0] += V_COST_COEFF * CppAD::pow(vars[V_START_IDX + t] - REF_V, 2);
    }

    // Minimize change-rate
    for (int t = 0; t < TIMESTEPS - 1; t++) {
      fg[0] += DELTA_COST_COEFF * CppAD::pow(vars[DELTA_START_IDX + t], 2);
      fg[0] += DELTA_COST_COEFF * CppAD::pow(vars[A_START_IDX + t], 2);
    }

    // Minimize the value gap between sequential actuations
    for (int t = 0; t < TIMESTEPS - 2; t++) {
      fg[0] += A_LOWER_COST_COEFF * CppAD::pow(vars[DELTA_START_IDX + t + 1] - vars[DELTA_START_IDX + t], 2);
      fg[0] += A_UPPER_COST_COEFF * CppAD::pow(vars[A_START_IDX + t + 1] - vars[A_START_IDX + t], 2);
    }

    //
    // Setup Constraints
    //

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`. This bumps up the position of all the other values.
    fg[1 + X_START_IDX] = vars[X_START_IDX];
    fg[1 + Y_START_IDX] = vars[Y_START_IDX];
    fg[1 + PSI_START_IDX] = vars[PSI_START_IDX];
    fg[1 + V_START_IDX] = vars[V_START_IDX];
    fg[1 + CTE_START_IDX] = vars[CTE_START_IDX];
    fg[1 + EPSI_START_IDX] = vars[EPSI_START_IDX];

    // The rest of the constraints
    for (int t = 0; t < TIMESTEPS - 1; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[X_START_IDX + t + 1];
      AD<double> y1 = vars[Y_START_IDX + t + 1];
      AD<double> psi1 = vars[PSI_START_IDX + t + 1];
      AD<double> v1 = vars[V_START_IDX + t + 1];
      AD<double> cte1 = vars[CTE_START_IDX + t + 1];
      AD<double> epsi1 = vars[EPSI_START_IDX + t + 1];

      // The state at time t.
      AD<double> x0 = vars[X_START_IDX + t];
      AD<double> y0 = vars[Y_START_IDX + t];
      AD<double> psi0 = vars[PSI_START_IDX + t];
      AD<double> v0 = vars[V_START_IDX + t];
      AD<double> cte0 = vars[CTE_START_IDX + t];
      AD<double> epsi0 = vars[EPSI_START_IDX + t];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[DELTA_START_IDX + t];
      AD<double> a0 = vars[A_START_IDX + t];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * pow(x0, 2) + 2 * coeffs[2] * x0 + coeffs[1]);

      fg[2 + X_START_IDX + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * DT);
      fg[2 + Y_START_IDX + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * DT);
      fg[2 + PSI_START_IDX + t] = psi1 - (psi0 - v0 * delta0 / LF * DT);
      fg[2 + V_START_IDX + t] = v1 - (v0 + a0 * DT);
      fg[2 + CTE_START_IDX + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * DT));
      fg[2 + EPSI_START_IDX + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / LF * DT);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables. SHOULD BE 0 besides initial state.
  Dvector vars(NUM_VARS);
  for (int i = 0; i < NUM_VARS; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(NUM_VARS);
  Dvector vars_upperbound(NUM_VARS);

  // Set all non-actuators upper and lower limits to the max negative and positive values.
  for (int i = 0; i < DELTA_START_IDX; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
  for (int i = DELTA_START_IDX; i < A_START_IDX; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = A_START_IDX; i < NUM_VARS; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints. Should be 0 besides initial state.
  Dvector constraints_lowerbound(CONSTRAINTS);
  Dvector constraints_upperbound(CONSTRAINTS);
  for (int i = 0; i < CONSTRAINTS; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[X_START_IDX] = x;
  constraints_lowerbound[Y_START_IDX] = y;
  constraints_lowerbound[PSI_START_IDX] = psi;
  constraints_lowerbound[V_START_IDX] = v;
  constraints_lowerbound[CTE_START_IDX] = cte;
  constraints_lowerbound[EPSI_START_IDX] = epsi;

  constraints_upperbound[X_START_IDX] = x;
  constraints_upperbound[Y_START_IDX] = y;
  constraints_upperbound[PSI_START_IDX] = psi;
  constraints_upperbound[V_START_IDX] = v;
  constraints_upperbound[CTE_START_IDX] = cte;
  constraints_upperbound[EPSI_START_IDX] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but if you uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds. Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);

  // Cost
  auto cost = solution.obj_value;

  std::vector<double> result;
  result.push_back((double) ok);
  result.push_back(cost);

  result.push_back(solution.x[DELTA_START_IDX]);
  result.push_back(solution.x[A_START_IDX]);

  for (int i = 0; i < TIMESTEPS - 1; i++) {
    result.push_back(solution.x[X_START_IDX + i + 1]);
    result.push_back(solution.x[Y_START_IDX + i + 1]);
  }

  return result;
}
