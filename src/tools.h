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
  static VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians for radar measurement function.
  */
  static MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * Map estimates to radar measurement.
  */
  static VectorXd RadarMeasurementFn(const VectorXd& x_state);

  static constexpr float PI=3.14159265358979f;

};

#endif /* TOOLS_H_ */
