function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

x=old_pose(1);
y=old_pose(2);
theta=old_pose(3);

if length(old_pose) >=4
    w=old_pose(4);
else
    w=1;
end


vR=motion_vector(1);
vL=motion_vector(2);

L=read_only_vars.agent_drive.interwheel_dist;
dt=read_only_vars.sampling_period;

v=(vR+vL)/2;
omega=(vR-vL)/L;

x_new = x + v*cos(theta)*dt;
y_new = y + v*sin(theta)*dt;
theta_new = theta + omega*dt;

sigma_xy = 0.02 + 0.05*abs(v);
sigma_theta = 0.02 + 0.05*abs(omega);

x_new = x_new + sigma_xy * randn;
y_new = y_new + sigma_xy * randn;
theta_new = theta_new + sigma_theta*randn;

theta_new= atan2(sin(theta_new), cos(theta_new));

new_pose = [x_new, y_new, theta_new, w];



%new_pose = old_pose;

end

