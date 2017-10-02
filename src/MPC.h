#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
  public:
      MPC();

      virtual ~MPC();

      // It was obtained by measuring the radius formed by running the vehicle in the
      // simulator around in a circle with a constant steering angle and velocity on a
      // flat terrain.
      //
      // Lf was tuned until the the radius formed by the simulating the model
      // presented in the classroom matched the previous radius.
      //
      // This is the length from front to CoG that has a similar radius.
      const double Lf = 2.67;

      // Solve the model given an initial state and polynomial coefficients.
      // Return the first actuations.
      vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */