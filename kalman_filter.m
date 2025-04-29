
function [updated_state, updated_covariance] = kalman_filter(state, covariance, gyro_input, measurement, dt, Q, R)
    % Prediction step
    F = eye(6) + dt * [zeros(3), -skew(gyro_input);
                       zeros(3), zeros(3)];
    state_pred = F * state;
    
    covariance_pred = F * covariance * F' + Q;

    % Update step
    H = eye(6);
    y = measurement - H * state_pred;
    S = H * covariance_pred * H' + R;
    K = covariance_pred * H' * inv(S);
    state_updated = state_pred + K * y;
    covariance_updated = (eye(6) - K * H) * covariance_pred;

    % Output updated state and covariance
    updated_state = state_updated;
    updated_covariance = covariance_updated;
   
end

