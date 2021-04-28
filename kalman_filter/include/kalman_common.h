#ifndef _KALMAN_COMMON
#define _KALMAN_COMMON

#include <Eigen/Dense>
#include <cassert>

class KalmanFilter
{
private:
    /* data */
    Eigen::MatrixXd A, C, Q, R, P, K, P0;
    int m,n;
    double t0, t;
    double dt;
    bool initialized;
    Eigen::MatrixXd I;
    Eigen::VectorXd x_hat, x_hat_new;
    
public:
    KalmanFilter();
    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,        //system model matrix                  x_hat_new = A*x_hat + e
        const Eigen::MatrixXd& C,        //measurement model matrix             z = C*x_hat + e
        const Eigen::MatrixXd& Q,        //system noise covariance             
        const Eigen::MatrixXd& R,        //measurement noise covariance
        const Eigen::MatrixXd& P         //posteriori covariance
    );
    void init();
    void init(double start_time, const Eigen::VectorXd& x0);
    void predict_and_update(const Eigen::VectorXd& z);
    void predict_and_update(const Eigen::VectorXd& z, double dt);
    void predict_and_update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd& A);
    
    //return current state;
    Eigen::VectorXd get_state() {return x_hat;};
    //return current time;
    double get_time() {return t;};
};

KalmanFilter::KalmanFilter(double dt,
                           const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& C,
                           const Eigen::MatrixXd& Q,
                           const Eigen::MatrixXd& R,
                           const Eigen::MatrixXd& P)
    :dt(dt), A(A), C(C), Q(Q), R(R), P0(P), 
     m(C.rows()), n(A.rows()), initialized(false), I(Eigen::MatrixXd::Identity(n,n)),
     x_hat(n), x_hat_new(n)
{
};

KalmanFilter::KalmanFilter(){};

void KalmanFilter::init() {
    x_hat.setZero();
    P = P0;
    t0 = 0;
    t = t0;
    initialized = true;
}

void KalmanFilter::init(double start_time, const Eigen::VectorXd& x0) {
    x_hat = x0;
    t0 = start_time;
    t = t0;
    P = P0;
    initialized = true;
}

void KalmanFilter::predict_and_update(const Eigen::VectorXd& z) {
    assert(initialized);
    
    //predict
    x_hat_new = A * x_hat;
    P = A*P*A.transpose() + Q;

    //update
    auto y = z - C*x_hat_new;
    K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
    x_hat_new += K*y;
    P = (I - K*C)*P;
    x_hat = x_hat_new;

    t += dt;
}

void KalmanFilter::predict_and_update(const Eigen::VectorXd& z, double dt) {
    this->dt = dt;
    predict_and_update(z);
}

void KalmanFilter::predict_and_update(const Eigen::VectorXd& z, double dt, const Eigen::MatrixXd& A) {
    this->A = A;
    this->dt = dt;
    predict_and_update(z);
}




#endif 