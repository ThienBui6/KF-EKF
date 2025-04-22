#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

//=====================
// Kalman Filter (KF)
//=====================
class KalmanFilter {
public:
    VectorXd x;  // State vector [position; velocity]
    MatrixXd P;  // Covariance matrix
    MatrixXd F;  // State transition matrix
    MatrixXd H;  // Measurement matrix
    MatrixXd Q;  // Process noise covariance
    MatrixXd R;  // Measurement noise covariance

    KalmanFilter(int state_dim, int meas_dim) :
        x(VectorXd::Zero(state_dim)),
        P(MatrixXd::Identity(state_dim, state_dim)),
        F(MatrixXd::Identity(state_dim, state_dim)),
        H(MatrixXd::Zero(meas_dim, state_dim)),
        Q(MatrixXd::Identity(state_dim, state_dim)), // Initialize with appropriate values
        R(MatrixXd::Identity(meas_dim, meas_dim)) { // Initialize with appropriate values
        // Set up H matrix for measuring position (example)
        H.block(0, 0, meas_dim, meas_dim) = MatrixXd::Identity(meas_dim, meas_dim);
    }

    void predict(double dt) {
        // Update the state transition matrix F based on dt
        F(0, 3) = dt; // Assuming state vector is [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]
        F(1, 4) = dt;
        F(2, 5) = dt;

        // Predict the state
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const VectorXd& z) {
        VectorXd y = z - H * x; // Measurement residual
        MatrixXd S = H * P * H.transpose() + R; // Residual covariance
        MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain
        x += K * y; // Update state estimate
        P = (MatrixXd::Identity(P.rows(), P.cols()) - K * H) * P; // Update covariance
    }
};