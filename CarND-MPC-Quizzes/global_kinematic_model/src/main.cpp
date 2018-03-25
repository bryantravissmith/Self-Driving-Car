// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"

//
// Helper functions
//
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;

// TODO: Implement the global kinematic model.
// Return the next state.
//
// NOTE: state is [x, y, psi, v]
// NOTE: actuators is [delta, a]


Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double delta = actuators[0];
  double a = actuators[1];

  double x_next = x + v * cos(psi) * dt;
  double y_next = y + v * sin(psi) * dt;
  double psi_next = psi + v * delta * dt / 2.0;
  double v_next = v + a * dt;

  next_state[0] = x_next;
  next_state[1] = y_next;
  next_state[2] = psi_next;
  next_state[3] = v_next;

  return next_state;
}

int main() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  // should be [0.212132, 0.212132, 0.798488, 1.3]
  auto next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}
