function [x, y, theta] = gen_unicycle_endpoint(cur_theta, linear_x, angular_z, max_time, resolution, eps, offset)

if nargin < 7
    offset = 0;
end
dt = 0.01;
max_num_steps = round(max_time / dt); 
%min_num_steps = round(max_num_steps/3.); 
min_num_steps = 0;
target_pose = zeros(max_num_steps-min_num_steps,3);
difference_to_lattice = zeros(max_num_steps-min_num_steps,1);

new_x = 0;
new_y = 0;
new_theta = cur_theta;

min_length = (1 - eps) * (max_time * abs(linear_x));
max_length = (1 + eps) * (max_time * abs(linear_x));

pose_idx = 0;
for ind = 1:max_num_steps
    new_x = new_x + dt * cos(new_theta + angular_z * dt/2.) * linear_x;
    new_y = new_y + dt * sin(new_theta + angular_z * dt/2.) * linear_x;
    new_theta = new_theta + angular_z * dt; 
    
    cur_length = sqrt(new_x^2 + new_y^2);
    
    if cur_length > min_length
        pose_idx = pose_idx + 1;

        target_pose(pose_idx, :) = [new_x, new_y, new_theta];
        difference_to_lattice_x = mod(new_x, resolution);
        difference_to_lattice_y = mod(new_y, resolution);
        difference_to_lattice(pose_idx) = sqrt(min(difference_to_lattice_x, resolution-difference_to_lattice_x) ^2 +  ...
        min(difference_to_lattice_y, resolution-difference_to_lattice_y) ^ 2);
    end
    if cur_length > max_length
       fprintf(1,'length exceeded');
       break
    end
end

[value, idx] = min(difference_to_lattice(1:pose_idx));


% hold on
% plot(target_pose(1:pose_idx,1), target_pose(1:pose_idx,2))
%plot(difference_to_lattice)
% [val, ind] = min(difference_to_lattice)
x = target_pose(idx,1);
y = target_pose(idx,2);

x = x + offset*cos(cur_theta) * sign(linear_x);
y = y + offset*sin(cur_theta) * sign(linear_x);


theta = target_pose(idx,3);

% match
mod_x = mod(x, resolution);
mod_y = mod(y, resolution);

if mod_x > resolution/2.
    approx_x = x + resolution - mod_x;
else
    approx_x = x - mod_x;
end

if mod_y > resolution/2.
    approx_y = y + resolution - mod_y;
else
    approx_y = y - mod_y;
end
    
x = approx_x;
y = approx_y;
% scatter(x, y);
% scatter(approx_x, approx_y);
% hold off


