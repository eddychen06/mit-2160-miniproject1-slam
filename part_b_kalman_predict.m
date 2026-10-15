function state_estimate_next = ...
    part_b_kalman_predict(state_estimate_prev,wheel_turns)
%PART_B_KALMAN_PREDICT Simulates the vehicle moving with certain wheel
%turns, updating the vehicle pose and pose covariance.

%% Unpacking the Structs
% Previous State Estimate
state_t = state_estimate_prev.state;%(mixed-unit vector)
P_t = state_estimate_prev.covariance;%(mixed-unit matrix)
state_dim = length(state_t); %(count)
x_t = state_t(1); %(m)
y_t = state_t(2); %(m)
th_t = state_t(3); %(rad)

% Wheel Turns
ds_r = wheel_turns.right; %(m)
ds_l = wheel_turns.left; %(m)

%% Define System Parameters
b = 0.3; %(m) wheel distance
k_r = 0.04; %(m) right wheel process noise variance factor
k_l = 0.04; %(m) left wheel process noise variance factor

%% State Estimate Simulation
v_t = (ds_r + ds_l) / 2;%(m)
dth = (ds_r - ds_l) / b;%(rad)

c_th = cos(th_t + dth/2);
s_th = sin(th_t + dth/2);

x_tp1 = x_t + v_t*c_th;
y_tp1 = y_t + v_t*s_th;
th_tp1 = th_t + dth;
        
%% Covariance Propagation
A_car = [
    1, 0, -v_t * s_th;
    0, 1, v_t * c_th;
    0, 0, 1;
];

A = eye(state_dim);
A(1:3, 1:3) = A_car;

G_car = [
    0.5 * c_th - (v_t/b)*s_th, 0.5*c_th + (v_t/b)*s_th;
    0.5 * s_th + (v_t/b)*c_th, 0.5*s_th - (v_t/b)*c_th;
    1/b, -1/b;
];

G = zeros(state_dim, 2);
G(1:3, :) = G_car;

Sigma_S = diag([k_r * abs(ds_r), k_l * abs(ds_l)]);

P_tp1 = A * P_t * A' + G * Sigma_S * G';

%% Repacking the Structs
state_tp1 = state_t;
state_tp1(1) = x_tp1;
state_tp1(2) = y_tp1;
state_tp1(3) = th_tp1;

state_estimate_next.state = state_tp1;
state_estimate_next.covariance = P_tp1;

end