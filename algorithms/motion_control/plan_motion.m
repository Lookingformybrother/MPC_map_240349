function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here
%%cv1 task 5
k = read_only_vars.counter;
public_vars.motion_vector = [0, 0];
% if k<60
%     public_vars.motion_vector=[1.0,1.0]; %rovně
% elseif k<120
%     public_vars.motion_vector=[0.9,1.0];
% elseif k<180
%     public_vars.motion_vector=[0.8,0.81];
% elseif k<240
%     public_vars.motion_vector=[1.0,0.9];
% elseif k<320
%     public_vars.motion_vector=[1.0,1.0];
% else
%     public_vars.motion_vector=[0,0];



% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

%public_vars.motion_vector = [0.5, 0.5];


end