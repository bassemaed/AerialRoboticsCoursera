function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
%Desired State %
yaw_des = des_state.yaw;
r_ddot_1_t = des_state.acc(1);
r_ddot_2_t = des_state.acc(2);
r_ddot_3_t = des_state.acc(3);
r_dot_1_t = des_state.vel(1);
r_dot_2_t = des_state.vel(2);
r_dot_3_t = des_state.vel(3);
r_1_t = des_state.pos(1);
r_2_t = des_state.pos(2);
r_3_t = des_state.pos(3);
p_des = 0;
q_des = 0;
r_des = des_state.yawdot;

%Actual State %
r_dot_1 = state.vel(1);
r_dot_2 = state.vel(2);
r_dot_3 = state.vel(3);
r_1 = state.pos(1);
r_2 = state.pos(2);
r_3 = state.pos(3);
phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

%Constants%
g = params.gravity;
m = params.mass;

%PD Controllers for u2 - eqn. 10 %
kp_phi = 10000;
kd_phi = 20;
kp_the = 10000;
kd_the = 20;
kp_psi = 0.5;
kd_psi = 0.5;

%PD Controllers for eqns. 11%
kp_1 = 100;
kd_1 = 10;
kp_2 = 100;
kd_2 = 10;
kp_3 = 100;
kd_3 = 10;

u_1 = 0;
u_2 = zeros(3,1);

%Equations 11a, 11b and 11c%
r_ddot_1_des = r_ddot_1_t + kd_1*(r_dot_1_t - r_dot_1) + kp_1*(r_1_t - r_1);
r_ddot_2_des = r_ddot_2_t + kd_2*(r_dot_2_t - r_dot_2) + kp_2*(r_2_t - r_2);
r_ddot_3_des = r_ddot_3_t + kd_3*(r_dot_3_t - r_dot_3) + kp_3*(r_3_t - r_3);

%Equations 14a, 14b and 16a %
phi_des = (1/g)*((r_ddot_1_des*sin(yaw_des))-(r_ddot_2_des*cos(yaw_des)));
the_des = (1/g)*((r_ddot_1_des*cos(yaw_des))+(r_ddot_2_des*sin(yaw_des)));
psi_des = yaw_des;

%Equation 13%
u_1 = m*g + m*r_ddot_3_des;
%Equation 10%
u_2(1) = kp_phi*(phi_des-phi) + kd_phi*(p_des - p);
u_2(2) = kp_the*(the_des-theta) + kd_the*(q_des - q);
u_2(3) = kp_psi*(psi_des-psi) + kd_psi*(r_des - r);

%Final Equations %
F = u_1;
M = params.I * u_2;
% =================== Your code ends here ===================

end
