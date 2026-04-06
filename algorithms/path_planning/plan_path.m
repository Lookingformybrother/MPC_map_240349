function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here
%tri testosvaci trajektorie: primka, kruhovy oblouk, sinusovka
%primka
path_type = 3;

if path_type==1
    x=linspace(2,18,30);
    y=2*ones(size(x));
    path=[x' y'];
elseif path_type == 2
    t=linspace(pi,pi/2,30);
    R=4;
    cx=6;
    cy=6;
    x=cx + R*cos(t);
    y=cy+R*sin(t);
    path=[x' y'];
elseif path_type==3
    x=linspace(2,18, 50);
    y=7+2*sin(0.6*x);
    path=[x' y'];
else
    path=[2 2;4 2;6 2];
end



% 
% 
% planning_required = 1;
% 
% if planning_required
%     
%     path = astar(read_only_vars, public_vars);
%     
%     path = smooth_path(path);
%     
% else
%     
%     path = public_vars.path;
%     
% end

end

