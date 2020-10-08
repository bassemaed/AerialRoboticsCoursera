function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;
m = params.mass;
g= params.gravity;
I = params.Ixx;

%Actual data
y_actual = state.pos(1);
z_actual = state.pos(2);
yd_actual = state.vel(1);
zd_actual = state.vel(2);
phi_actual = state.rot;
omega_actual = state.omega;
%Desired data
y_desired = des_state.pos(1);
z_desired = des_state.pos(2);
yd_desired = des_state.vel(1);
zd_desired = des_state.vel(2);
ydd_desired = des_state.acc(1);
zdd_desired = des_state.acc(2);
%error data
ep_z = z_desired - z_actual;
ed_z = zd_desired - zd_actual;
ep_y = y_desired - y_actual;
ed_y = yd_desired - yd_actual;
ed_phi =  - omega_actual;
%PD gains
kp_z = 80;
kd_z = 7.5;
kp_y = 12;
kd_y = 5;
kp_phi = 1600;
kd_phi  =26.666;
%Control Gains
u1 = m*(g+ zdd_desired + kd_z*ed_z + kp_z*ep_z);
phi_desired = (-1/g)*(ydd_desired + kd_y*ed_y +kp_y*ep_y);
ep_phi = phi_desired - phi_actual;
u2 = I*(kp_phi*ep_phi + kd_phi*ed_phi);
end

