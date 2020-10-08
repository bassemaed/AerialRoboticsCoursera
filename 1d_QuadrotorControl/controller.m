function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
m = params.mass;
g = params.gravity;
k_p = 60;
k_d = 10;
z__ = 0;
u = 0;
e = zeros(2,1);
e = s_des-s;
u = m*(k_p*e(1)+k_d*e(2)+g+z__);

% FILL IN YOUR CODE HERE


end

