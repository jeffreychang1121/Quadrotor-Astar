function [desired_state] = trajectory_generator(t, qn, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In the framework,
% this function will first (and only once!) be called like this:
%
% trajectory_generator([],[], 0, path)
%
% i.e. map = varargin{1} and path = varargin{2}.
%
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a
% point in the path. N is the total number of points in the path
%
% This is when you compute and store the trajectory.
%
% Later it will be called with only t and qn as an argument, at
% which point you generate the desired state for point t.
%

desired_state = [];

% use the "persistent" keyword to keep your trajectory around
% inbetween function calls

quad_speed = 2.0;

persistent coef goal t_path t_total;

% if input more than 2
if nargin > 2
    
    
    
    path2 = varargin{2};
    path = path2{1,1};
    num_path = size(path,1);
    num_seg = num_path - 1;
    % goal point
    goal = path(end,:);
    % spline coefficients
%     coef = zeros(3,6,num_seg);
    coef = zeros(3,4,num_seg);
    % time at each point
    t_path = zeros(num_path,1);
    % time spent for each segment
    duration = zeros(num_seg,1);
    
    path_total = 0;
    
    for i = 1 : num_seg
        duration(i) = norm(path(i+1,:) - path(i,:));
        path_total = path_total + duration(i);
    end
    
    t_total = path_total / quad_speed;
    
    % distribute time according to segment length
    duration = sqrt(duration) / sum(sqrt(duration)) * t_total;
    
    t_path(1) = 0;
    for i = 2 : num_path
        t_path(i) = t_path(i-1) + duration(i-1);
    end
    
    % initial condition for velocity
    init_vel = zeros(num_path, 3);
    
    % initial velocity ratio
    speed = .7;
    %cur_vel = [0,0,0];
    new_vel = (path(2,:) - path(1,:)) / norm(path(2,:) - path(1,:));
    init_vel(1,:) = speed * (new_vel);
    for i = 2 : num_path - 1
        %pre_vel = cur_vel;
        cur_vel = new_vel;
        new_vel = (path(i+1,:) - path(i,:)) / norm(path(i+1,:) - path(i,:));
        angle = atan2(norm(cross(cur_vel,new_vel)), dot(cur_vel,new_vel));
        if angle > pi / 2
            init_vel(i,:) = speed * (new_vel + cur_vel) / 2;
        else
            init_vel(i,:) = speed * (new_vel + cur_vel) / 2;
            
        end
    end
   
    for i = 1 : num_path - 1
        
%         A = [
%             1, t_path(i), t_path(i)^2, t_path(i)^3, t_path(i)^4, t_path(i)^5;
%             1, t_path(i+1), t_path(i+1)^2, t_path(i+1)^3, t_path(i+1)^4, t_path(i+1)^5;
%             0, 1, 2*t_path(i), 3*t_path(i)^2, 4*t_path(i)^3, 5*t_path(i)^4;
%             0, 1, 2*t_path(i+1), 3*t_path(i+1)^2, 4*t_path(i+1)^3, 5*t_path(i+1)^4;
%             0, 0, 2, 6*t_path(i), 12*t_path(i)^2, 20*t_path(i);
%             0, 0, 2, 6*t_path(i+1), 12*t_path(i+1)^2, 20*t_path(i+1);
%             ];

        A = [
            1, t_path(i), t_path(i)^2, t_path(i)^3;
            1, t_path(i+1), t_path(i+1)^2, t_path(i+1)^3;
            0, 1, 2*t_path(i), 3*t_path(i)^2;
            0, 1, 2*t_path(i+1), 3*t_path(i+1)^2;
            ];
        
        % coefficient for x,y,z
        for j = 1 : 3
%             B = [
%                 path(i,j);
%                 path(i+1,j);
%                 0;
%                 0;
%                 0;
%                 0;
%                 ];

            B = [
                path(i,j);
                path(i+1,j);
                0;
                0;
                %init_vel(i,j);
                %init_vel(i+1,j);
                ];
            
            % coef([x,y,z],six coefficient,path)
            coef(j,:,i) = (A \ B)';
        end
    end
    
else
    if t > t_total
        pos = goal;
        vel = [0, 0, 0];
        acc = [0, 0, 0];
    else
        % determine which time interval's coefficient should be called
        for i = 2 : size(t_path,1)
            t_cur = t_path(i,1);
            if t < t_cur
                cur_seg = i - 1;
                break;
            end
        end
        
        coef_x = coef(1,:,cur_seg);
        coef_y = coef(2,:,cur_seg);
        coef_z = coef(3,:,cur_seg);
        
%         x_pos = [1, t, t^2, t^3, t^4, t^5] * coef_x';
%         y_pos = [1, t, t^2, t^3, t^4, t^5] * coef_y';
%         z_pos = [1, t, t^2, t^3, t^4, t^5] * coef_z';
%         
%         x_vel = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4] * coef_x';
%         y_vel = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4] * coef_y';
%         z_vel = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4] * coef_z';
%         
%         x_acc = [0, 0, 2, 6*t, 12*t^2, 20*t^3] * coef_x';
%         y_acc = [0, 0, 2, 6*t, 12*t^2, 20*t^3] * coef_y';
%         z_acc = [0, 0, 2, 6*t, 12*t^2, 20*t^3] * coef_z';
        
        x_pos = [1, t, t^2, t^3] * coef_x';
        y_pos = [1, t, t^2, t^3] * coef_y';
        z_pos = [1, t, t^2, t^3] * coef_z';
        
        x_vel = [0, 1, 2*t, 3*t^2] * coef_x';
        y_vel = [0, 1, 2*t, 3*t^2] * coef_y';
        z_vel = [0, 1, 2*t, 3*t^2] * coef_z';
        
        x_acc = [0, 0, 2, 6*t] * coef_x';
        y_acc = [0, 0, 2, 6*t] * coef_y';
        z_acc = [0, 0, 2, 6*t] * coef_z';
        
        pos = [x_pos, y_pos, z_pos];
        vel = [x_vel, y_vel, z_vel];
        acc = [x_acc, y_acc, z_acc];
        
    end
    
    
    %
    % When called without varargin (isempty(varargin) == true), compute
    % and return the desired state here.
    %
    
    yaw = 0;
    yawdot = 0;
    
    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
    
end
end
