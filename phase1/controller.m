function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================
% ...
% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

%% Gains
kp_x = 10;
kp_y = 10;
kp_z = 20;

kd_x = kp_x * 0.5;
kd_y = kp_y * 0.5;
kd_z = kp_z * 0.5;

Kp = [ kp_x, 0, 0;
       0, kp_y, 0;
       0, 0, kp_z];
 
Kd = [ kd_x, 0, 0;
       0, kd_y, 0;
       0, 0, kd_z];
   
kr_x = 2500;
kr_y = 2500;
kr_z = 500;

kw_x = 100;
kw_y = 100;
kw_z = 50;

Kr = [ kr_x, 0, 0;
       0, kr_y, 0;
       0, 0, kr_z];
   
Kw = [ kw_x, 0, 0;
       0, kw_y, 0;
       0, 0, kw_z];


% command r_dotdot
r_dotdot_des = qd{qn}.acc_des - Kd * (qd{qn}.vel - qd{qn}.vel_des) ...
                - Kp * (qd{qn}.pos - qd{qn}.pos_des);
% command force
F_des = params.mass * r_dotdot_des + [0; 0; params.mass * params.grav];

% rotation matrix
R = eulzxy2rotmat(qd{qn}.euler);
b3 = R * [0; 0; 1];

% force u1
u1 = b3' * F_des;

% b3_des aligned across F_des
% bi_des orthogonal
b3_des = F_des / norm(F_des);
a_psi = [cos(qd{qn}.yaw_des); sin(qd{qn}.yaw_des); 0];
b2_des = cross(b3_des, a_psi) / norm(cross(b3_des, a_psi));
b1_des = cross(b2_des, b3_des);

% determine R_des
R_des = [b1_des, b2_des, b3_des];

% error vector
errv = 0.5 * veemap(R_des' * R - R' * R_des);

% desire angles
eul = rotmat2eulzxy(R_des);
phi_des = eul(1);
theta_des = eul(2);
psi_des = eul(3);

% error angular velocity
errw = qd{qn}.omega - 0;

% force u2
u2 = params.I * (- Kr * errv' - Kw * errw);

u    = [u1; u2];   % control input u, you should fill this in
                  
% Thrust
F    = u(1);       % This should be F = u(1) from the project handout

% Moment
M    = u(2:4);     % note: params.I has the moment of inertia

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
