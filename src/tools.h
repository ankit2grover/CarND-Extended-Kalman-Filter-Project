#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  // From: http://stackoverflow.com/a/29871193/1321129
  /* change to `float/fmodf` or `long double/fmodl` or `int/%` as appropriate */
  /* wrap x -> [0,max) */
  double wrapMax(double x, double max);

  /* wrap x -> [min,max) */
  double wrapMinMax(double x, double min, double max);

};

#endif /* TOOLS_H_ */
