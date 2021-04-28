#include "kalman_common.h"
#include <iostream>
#include <time.h>


int main(int argc, char** argv) {
    int state_num = 7;
    int measurement_size = 7;

    double dt = 1.0/30; 

    Eigen::MatrixXd A(state_num, state_num);
    Eigen::MatrixXd C(measurement_size, state_num);
    Eigen::MatrixXd Q(state_num, state_num);
    Eigen::MatrixXd R(measurement_size, measurement_size);
    Eigen::MatrixXd P0(state_num, state_num);

    A = Eigen::MatrixXd::Identity(state_num, state_num);
    C = Eigen::MatrixXd::Identity(measurement_size, measurement_size);
    Q << Eigen::MatrixXd::Identity(state_num, state_num) * 0.1;
    R << Eigen::MatrixXd::Identity(measurement_size, measurement_size) * 0.02;
    //initial measurement covariance P0 // increase
    P0 << 0.05, 0.005, 0.005, 0.0, 0.0, 0.0, 0.0,
          0.005, 0.05, 0.005, 0.0, 0.0, 0.0, 0.0,
          0.005, 0.005, 0.05, 0.0, 0.0, 0.0, 0.0,
          0.0,   0.0,   0.0,  0.05, 0.005, 0.005, 0.005,
          0.0,   0.0,   0.0,  0.005, 0.05, 0.005, 0.005,
          0.0,   0.0,   0.0,  0.005, 0.005, 0.05, 0.005,
          0.0,   0.0,   0.0,  0.005, 0.005, 0.005, 0.05;



    std::cout <<"A: \n" << A << std::endl;
    std::cout <<"C: \n" << C << std::endl;
    std::cout <<"Q: \n" << Q << std::endl;
    std::cout <<"R: \n" << R << std::endl;
    std::cout <<"P: \n" << P0 << std::endl;

    KalmanFilter kf(dt, A, C, Q, R, P0);

    //get measurements here
    Eigen::Matrix<double, 5, 7> measurements_t;
    measurements_t << 0.719425, -0.008684, 0.503272, 0.701833, 0.712331, -0.000610, 0.003994,
                      0.728202,  0.005453, 0.477658, 0.697892, 0.715364, -0.030374, 0.016693,
                      0.727990,  0.006200, 0.494247, 0.691623, 0.718666, -0.036695, 0.048912,
                      0.733156,  0.005131, 0.489180, 0.687127, 0.723818, -0.021658, 0.058946,
                      0.735768, -0.011198, 0.480659, 0.697859, 0.714692,  0.046429, 0.007293;
    
    Eigen::Matrix<double, 7, 5> measurements = measurements_t.transpose();

    //initial guess
    Eigen::VectorXd x0(state_num);
    x0 << 0.5, 0.0, 0.50, 0.70, 0.71, 0.0, 0.00;
    kf.init(0, x0);

    std::cout <<"Initial guess of state is :\n" << kf.get_state().transpose() <<std::endl;
    for (int i=0; i < measurements.cols(); i++) {
        kf.predict_and_update(measurements.col(i));
        std::cout << "Time: "<<kf.get_time() << std::endl;
        std::cout << "Updated with: \n" << measurements.col(i).transpose() <<std::endl;
        std::cout << "New state is: \n" << kf.get_state().transpose() << std::endl;
        std::cout << "=======================================" << std::endl;
    }

    


    return 0;
}