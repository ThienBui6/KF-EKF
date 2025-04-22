// Extended Kalman Filter (EKF)
//=====================
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

//=====================
// Extended Kalman Filter (EKF)
//=====================
class ExtendedKalmanFilter {
public:
    VectorXd x;  // State vector [x, y, z, vx, vy, vz]
    MatrixXd P;  // Covariance matrix
    MatrixXd Q;  // Process noise covariance
    MatrixXd R;  // Measurement noise covariance
    double mu = 3.986e14;  // Earth gravitational parameter

    ExtendedKalmanFilter() :
        x(VectorXd::Zero(6)),
        P(MatrixXd::Identity(6, 6)),
        Q(MatrixXd::Identity(6, 6) * 1e-4),
        R(MatrixXd::Identity(3, 3) * 1e2) {}

    VectorXd dynamics(const VectorXd& state, double dt) {
        // Simplified orbital dynamics (two-body problem)
        VectorXd new_state(6);
        Vector3d pos = state.head(3);
        Vector3d vel = state.segment(3, 3);
        
        double r = pos.norm();
        Vector3d accel = -mu * pos / pow(r, 3);
        
        new_state.head(3) = pos + vel * dt;
        new_state.segment(3, 3) = vel + accel * dt;
        return new_state;
    }

    MatrixXd compute_jacobian(const VectorXd& state, double dt) {
        // Jacobian of dynamics model
        MatrixXd F(6, 6);
        F.setIdentity();
        
        Vector3d pos = state.head(3);
        double r = pos.norm();
        
        // Ensure r is not zero to avoid division by zero
        if (r > 0) {
            Matrix3d daccel = -mu * (Matrix3d::Identity() / pow(r, 3) - 3 * pos * pos.transpose() / pow(r, 5));
            
            F.block(0, 3, 3, 3) = Matrix3d::Identity() * dt;
            F.block(3, 0, 3, 3) = daccel * dt;
        }
        
        return F;
    }

    void predict(double dt) {
        // Nonlinear prediction step
        x = dynamics(x, dt);
        MatrixXd F_jac = compute_jacobian(x, dt);
        P = F_jac * P * F_jac.transpose() + Q;
    }

    void update(const VectorXd& z) {
        // Measurement update (position measurements)
        MatrixXd H(3, 6);
        H << Matrix3d::Identity(), Matrix3d::Zero();
        
        VectorXd y = z - H * x; // Measurement residual
        MatrixXd S = H * P * H.transpose() + R; // Residual covariance
        
        // Check for singularity
        if (S.determinant() == 0) {
            std::cerr << "Error: Measurement covariance matrix is singular!" << std::endl;
            return; // Handle singularity appropriately
        }
        
        MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain
        
        x += K * y; // Update state estimate
        P = (MatrixXd::Identity(P.rows(), P.cols()) - K * H) * P; // Update covariance
    }
};

// Example usage
int main() {
    // EKF Example
    ExtendedKalmanFilter ekf;
    
    // Initial state (position and velocity)
    ekf.x << 7e6, 0, 0, 0, 7.5e3, 0;  // Example initial state: [x, y, z, vx, vy, vz]
    
    // Example measurement (position)
    Vector3d measurement(7e6, 1e3, 0);  // Example LEO position measurement
    
    // Prediction and update cycle
    double dt = 1.0;  // 1 second time step
    ekf.predict(dt);  // Perform prediction
    ekf.update(measurement);  // Perform update with measurement
    
    // Output the updated state
    std::cout << "Updated state:\n" << ekf.x << std::endl;
    std::cout << "Updated covariance:\n" << ekf.P << std::endl;

    return 0;
}