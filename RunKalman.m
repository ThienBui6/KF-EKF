% Define parameters
dt = 0.1; % Time step
Q = 0.001 * eye(6); % Process noise covariance
R = 0.1 * eye(6); % Measurement noise covariance

% Initial state estimate [attitude angles; angular velocities]
initial_state = [1; 1; 1; 1; 1; 1];

% Initial covariance estimate
initial_covariance = eye(6);

% Number of time steps
num_steps = 100;

% Preallocate arrays to store simulated states and measurements
states = zeros(6, num_steps);
measurements = zeros(6, num_steps);

% Simulate gyro inputs (angular velocity)
gyro_inputs = randn(3, num_steps) * 0.01; % Random noise added to simulate real-world data

% Simulate initial true state
true_state = initial_state;
estimated_state= initial_state*0;
states(:, 1) = estimated_state;

estimated_states(:,1)=estimated_state;
true_states(:,1)=true_state;

% Simulate Kalman filter
for i = 2:num_steps
    % Simulate measurement (attitude and angular velocity) from true state
    measurement_noise =  sqrtm(R)*randn(6, 1);
    measurement = [true_state(1:3) + measurement_noise(1:3); true_state(4:6) + measurement_noise(4:6)];
    measurements(:, i) = measurement;

    % Run Kalman filter
    [updated_state, ~] = kalman_filter(true_state, initial_covariance, gyro_inputs(:, i), measurement, dt, Q, R);
    
    % Store true state, measurement, and estimated state
    true_state = updated_state;
    true_states(:, i+1) = true_state;
    estimated_states(:, i) = updated_state;
end

% Plot results
t = (0:num_steps-1) * dt;
figure;
subplot(2, 1, 1);
plot(t, estimated_states(1:3, 1:num_steps));
xlabel('Time (s)');
ylabel('Attitude (rad)');
title('Estimated Attitude');

subplot(2, 1, 2);
plot(t, estimated_states(4:6, 1:num_steps));
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Estimated Angular Velocity');

figure;
subplot(2, 1, 1);
plot(t, measurements(1:3, 1:num_steps));
xlabel('Time (s)');
ylabel('Attitude (rad)');
title('Measured Attitude');

subplot(2, 1, 2);
plot(t, measurements(4:6, 1:num_steps));
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Measured Angular Velocity');

figure;
subplot(3, 1, 1);
plot(t, estimated_states(1, 1:num_steps),'b',t,true_states(1, 1:num_steps),'r');
xlabel('Time (s)');
ylabel('Roll (rad)');
title('Estimated (blue) and true (red) Attitude');

subplot(3, 1, 2);
plot(t, estimated_states(2, 1:num_steps),'b',t,true_states(2, 1:num_steps),'r');
xlabel('Time (s)');
ylabel('Pitch (rad)');
title('Estimated (blue) and true Attitude (red)');
subplot(3, 1, 3);
plot(t, estimated_states(3, 1:num_steps),'b',t,true_states(3, 1:num_steps),'r');
xlabel('Time (s)');
ylabel('Yaw (rad)');
title('Estimated (blue) and true (red) Attitude');