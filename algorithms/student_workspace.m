function [public_vars] = student_workspace(read_only_vars, public_vars)
%STUDENT_WORKSPACE Semester project

if ~isfield(public_vars, "motion_vector")
    public_vars.motion_vector = [0, 0];
end
if ~isfield(public_vars, "path")
    public_vars.path = [];
end
if ~isfield(public_vars, "project") || isempty(public_vars.project)
    public_vars.project = struct();
end

% uloz map limits pro estimate_pose
public_vars.project.map_limits = read_only_vars.map.limits;

% default fields
if ~isfield(public_vars.project, "mode")
    public_vars.project.mode = "localize";
end
if ~isfield(public_vars.project, "pose_history")
    public_vars.project.pose_history = [];
end
if ~isfield(public_vars.project, "escape_steps")
    public_vars.project.escape_steps = 0;
end
if ~isfield(public_vars.project, "near_goal_counter")
    public_vars.project.near_goal_counter = 0;
end
if ~isfield(public_vars.project, "relocalize_cooldown")
    public_vars.project.relocalize_cooldown = 0;
end
if ~isfield(public_vars.project, "relocalizing")
    public_vars.project.relocalizing = false;
end

% ===== 8. Initialization =====
if (read_only_vars.counter == 1)

    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

    public_vars.project.mode = "localize";
    public_vars.project.pose_history = [];
    public_vars.project.escape_steps = 0;
    public_vars.project.near_goal_counter = 0;
    public_vars.project.relocalize_cooldown = 0;
    public_vars.project.relocalizing = false;

    if isfield(public_vars, "target_idx")
        public_vars.target_idx = 1;
    end
end

% ===== 9. Particle filter =====
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% ===== 10. Kalman filter =====
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% ===== 11. Estimate pose =====
public_vars.estimated_pose = estimate_pose(public_vars); % [x,y,theta]

% Ulozeni historie polohy
if all(isfinite(public_vars.estimated_pose))
    public_vars.project.pose_history = [public_vars.project.pose_history; public_vars.estimated_pose];
end

% drz jen poslednich 120 vzorku
if size(public_vars.project.pose_history,1) > 120
    public_vars.project.pose_history = public_vars.project.pose_history(end-119:end,:);
end

% Min lidar
min_lidar = inf;
if isfield(read_only_vars, "lidar_distances") && ~isempty(read_only_vars.lidar_distances)
    vals = read_only_vars.lidar_distances(isfinite(read_only_vars.lidar_distances));
    if ~isempty(vals)
        min_lidar = min(vals);
    end
end

% GNSS dostupnost
gnss_available = false;
if isfield(read_only_vars, "gnss_position") && ~isempty(read_only_vars.gnss_position)
    gnss_available = ~any(isnan(read_only_vars.gnss_position));
end

% PF confidence
[pf_ok, pf_stats] = pf_is_confident(public_vars);

% Kontrola: estimated pose mimo mapu?
lims = read_only_vars.map.limits;
out_of_bounds = false;
if all(isfinite(public_vars.estimated_pose(1:2)))
    x = public_vars.estimated_pose(1);
    y = public_vars.estimated_pose(2);
    out_of_bounds = (x < lims(1)) || (x > lims(3)) || (y < lims(2)) || (y > lims(4));
end

% Falesny cil
goal_xy = read_only_vars.map.goal(1:2);
near_goal_radius = 0.35;
near_goal_steps = 35;

if public_vars.project.relocalize_cooldown > 0
    public_vars.project.relocalize_cooldown = public_vars.project.relocalize_cooldown - 1;
end

near_goal = false;
if all(isfinite(public_vars.estimated_pose(1:2)))
    near_goal = norm(public_vars.estimated_pose(1:2) - goal_xy) < near_goal_radius;
end

if near_goal
    public_vars.project.near_goal_counter = public_vars.project.near_goal_counter + 1;
else
    public_vars.project.near_goal_counter = 0;
end

if public_vars.project.near_goal_counter >= near_goal_steps && public_vars.project.relocalize_cooldown == 0
    disp("RELOCALIZATION RESET: FALSE GOAL")

    public_vars = init_particle_filter(read_only_vars, public_vars);

    public_vars.path = [];
    public_vars.motion_vector = [0, 0];

    if isfield(public_vars, "target_idx")
        public_vars.target_idx = 1;
    end

    public_vars.project.mode = "escape";
    public_vars.project.pose_history = [];
    public_vars.project.escape_steps = 0;
    public_vars.project.near_goal_counter = 0;
    public_vars.project.relocalize_cooldown = 80;
    public_vars.project.relocalizing = true;

    if isfield(public_vars, "mu")
        public_vars.mu = [NaN; NaN; NaN];
    end
    if isfield(public_vars, "sigma")
        public_vars.sigma = eye(3);
    end
end

% Stuck detekce
stuck = false;
moved = NaN;
box_x = NaN;
box_y = NaN;

window_len = 15;
move_thresh = 0.45;
box_thresh = 0.55;

if size(public_vars.project.pose_history,1) >= window_len
    H = public_vars.project.pose_history(end-window_len+1:end, 1:2);

    moved = norm(H(end,:) - H(1,:));
    box_x = max(H(:,1)) - min(H(:,1));
    box_y = max(H(:,2)) - min(H(:,2));

    stuck = (moved < move_thresh) && (box_x < box_thresh) && (box_y < box_thresh);
end

% Trigger pro docasny escape
too_close = min_lidar < 0.22;

% Kdyz jsem zaseknuty nebo mimo mapu, tak zahod lokalizaci a zacni znovu
if (stuck || out_of_bounds) && ~public_vars.project.relocalizing && public_vars.project.relocalize_cooldown == 0

    disp("RELOCALIZATION RESET: STUCK nebo OUT OF MAP")

    public_vars = init_particle_filter(read_only_vars, public_vars);

    public_vars.path = [];
    public_vars.motion_vector = [0, 0];

    if isfield(public_vars, "target_idx")
        public_vars.target_idx = 1;
    end

    public_vars.project.mode = "escape";
    public_vars.project.pose_history = [];
    public_vars.project.escape_steps = 0;
    public_vars.project.near_goal_counter = 0;
    public_vars.project.relocalize_cooldown = 60;
    public_vars.project.relocalizing = true;

    if isfield(public_vars, "mu")
        public_vars.mu = [NaN; NaN; NaN];
    end
    if isfield(public_vars, "sigma")
        public_vars.sigma = eye(3);
    end
end

% Rezimy
if public_vars.project.relocalizing
    % Po resetu lokalizace jed jen podle lidaru, dokud se PF znovu nechyti
    public_vars.project.mode = "escape";

    % Jakmile je PF znovu rozumny a odhad je uvnitr mapy, vrat se zpet
    if pf_ok && ~out_of_bounds && min_lidar > 0.25
        public_vars.project.relocalizing = false;
        public_vars.project.mode = "goal";
        public_vars.path = [];
        if isfield(public_vars, "target_idx")
            public_vars.target_idx = 1;
        end
    end

else
    % Docasny nouzovy escape, kdyz jsme moc blizko zdi
    if too_close
        public_vars.project.mode = "escape";
        public_vars.project.escape_steps = 12;

    elseif strcmp(public_vars.project.mode, "escape") && public_vars.project.escape_steps > 0
        public_vars.project.escape_steps = public_vars.project.escape_steps - 1;

    else
        if gnss_available || pf_ok
            public_vars.project.mode = "goal";
        else
            public_vars.project.mode = "localize";
        end
    end
end

% ===== 12. Path planning =====
public_vars.path = plan_path(read_only_vars, public_vars);

% ===== 13. Motion planning =====
public_vars = plan_motion(read_only_vars, public_vars);

% Debug
if mod(read_only_vars.counter, 20) == 0
    disp("----- PROJECT MODE DEBUG -----")
    disp("mode:")
    disp(public_vars.project.mode)
    disp("relocalizing:")
    disp(public_vars.project.relocalizing)
    disp("min_lidar:")
    disp(min_lidar)
    disp("too_close:")
    disp(too_close)
    disp("out_of_bounds:")
    disp(out_of_bounds)
    disp("stuck:")
    disp(stuck)
    disp("escape_steps:")
    disp(public_vars.project.escape_steps)
    disp("gnss available:")
    disp(gnss_available)
    disp("pf confident:")
    disp(pf_ok)
    disp("pf spread [x y theta]:")
    disp(pf_stats)

    if ~isnan(moved)
        disp("moved:")
        disp(moved)
        disp("box_x:")
        disp(box_x)
        disp("box_y:")
        disp(box_y)
    end
end

end