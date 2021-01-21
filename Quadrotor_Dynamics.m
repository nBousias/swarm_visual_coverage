function [state_dot] = Quadrotor_Dynamics(sim,x,u)
% calculates the state derivatives from the nonlinear quadrotor dynamics
% INPUTS
% x is 12x1 state vector
% u is 4x1 control vector
% OUTPUTS
% state_dot is 12x1 state derivative

m      = sim.m;
g      = sim.gr;
I      = sim.J;
states = sim.states;
inv_I  = sim.inv_inertia;

%_________________ get input variables
x_dot     = x(4);
y_dot     = x(5);
z_dot     = x(6);
phi       = x(7);
theta     = x(8);
psi       = x(9);
phi_dot   = x(10);
theta_dot = x(11);
psi_dot   = x(12);

%______________ assemble state_dot____________

state_dot = zeros(states,1);

%----- translational
% position_dot
state_dot(1) = x_dot;
state_dot(2) = y_dot;
state_dot(3) = z_dot;

% position_ddot
r_ddot=[0;0;-g]+(1/m)*rotation_matrix(phi,theta,psi)*[0;0;u(1)];
state_dot(4) = r_ddot(1);
state_dot(5) = r_ddot(2);
state_dot(6) = r_ddot(3);

%---rotational
% angle_dot
state_dot(7) = phi_dot;
state_dot(8) = theta_dot;
state_dot(9) = psi_dot;

% angle_ddot
a_dot=[phi_dot; theta_dot; psi_dot];
a_ddot = inv_I*( u(2:4)- cross(a_dot,I*a_dot) );
state_dot(10) = a_ddot(1);
state_dot(11) = a_ddot(2);
state_dot(12) = a_ddot(3);

end

