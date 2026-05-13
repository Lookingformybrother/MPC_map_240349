function [goal_xy] = find_escape_goal(read_only_vars, public_vars)
% Najde docasny cil do volnejsiho prostoru podle LiDARu

goal_xy = [];

if ~isfield(public_vars.project, "escape_total_steps")
    public_vars.project.escape_total_steps = 0;
end
public_vars.project.escape_total_steps = public_vars.project.escape_total_steps + 1;

if ~isfield(public_vars, "estimated_pose") || isempty(public_vars.estimated_pose) ...
        || any(isnan(public_vars.estimated_pose(1:2)))
    return
end

if ~isfield(read_only_vars, "lidar_distances") || isempty(read_only_vars.lidar_distances)
    return
end

pose = public_vars.estimated_pose;
lidar_distances = read_only_vars.lidar_distances(:);
lidar_config = read_only_vars.lidar_config(:);

valid = isfinite(lidar_distances);
if ~any(valid)
    return
end

% vyber smer s nejvetsi volnou vzdalenosti
valid_idx = find(valid);
[~, local_idx] = max(lidar_distances(valid));
best_idx = valid_idx(local_idx);

best_dist = lidar_distances(best_idx);
best_dir = pose(3) + lidar_config(best_idx);

% chceme popojet do volneho prostoru, ale ne uplne daleko
move_dist = min(1.0, max(0.6, 0.5 * best_dist));

candidate = pose(1:2) + move_dist * [cos(best_dir), sin(best_dir)];

% omez do mapy s malou rezervou
limits = read_only_vars.map.limits; % [xmin ymin xmax ymax]
margin = 0.8;

candidate(1) = max(limits(1) + margin, min(limits(3) - margin, candidate(1)));
candidate(2) = max(limits(2) + margin, min(limits(4) - margin, candidate(2)));

goal_xy = candidate;

if public_vars.project.escape_total_steps > 2
    public_vars.project.mode = "localize";
    public_vars.project.escape_total_steps = 0;
    public_vars.path = [];
end

end