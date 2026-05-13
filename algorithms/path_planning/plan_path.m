function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH

if isfield(public_vars, "project") && isfield(public_vars.project, "mode")
    if strcmp(public_vars.project.mode, "escape")
        path = [];
        return
    end
end

planning_required = 1;

if planning_required
    path = astar(read_only_vars, public_vars);
    path = smooth_path(path);
else
    path = public_vars.path;
end

end