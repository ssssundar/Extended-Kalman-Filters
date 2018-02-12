#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {


  unsigned int est_size = estimations.size();
  unsigned int ground_truth_size = ground_truth.size(); 
  //cout << "RMSE: est_size:" << est_size << "grnd_truth_size:" 
    //   << ground_truth_size << endl; 
 
  // In the case any error in this function, default rmse value is used to 
  // return from this function. 
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Check the validity of the following inputs:
  //  * the estimation vector size should not be 0
  //  * the estimation vector size should eual ground truth vector size
  if( (est_size != ground_truth_size ) ||
       (est_size == 0 ) ) { 
     
    cout << "Invalid estimation size: " << est_size << 
            "OR ground_truth size: " << ground_truth_size << endl;
    return rmse;

  }

  // Accumulate squared residuals

  //cout << "RMSE: before for" << endl;
 
 for( unsigned int i=0; i < est_size; ++i ) {
    
    VectorXd residual = estimations[i] - ground_truth[i];
    
    // Coefficient-wise multiplication
    residual = residual.array() * residual.array();

    rmse += residual;

  }

  //cout << "RMSE: after for" << rmse << endl;

  // Calculate the mean
  rmse = rmse/est_size;

  //cout << "RMSE: after div" << endl;

  // Calcuate the squared root
  rmse = rmse.array().sqrt();

  // Retune the calculated rmse
  return rmse;

}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  
  //cout << "Entering Jacobian" << endl;

  MatrixXd Hj(3,4);

  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);


  // Pre-compute a set of terms to avoid repeated computation
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  // Check division by 0
  if( (fabs(c1)  < 0.0001) ) {

    cout << "CalculateJacobian() - Error - Division by Zero" << endl;
    return Hj;  
  }

  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*( vx*py - vy*px)/c3, px*( px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj; 

}

