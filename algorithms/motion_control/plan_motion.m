function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION
% Normalni jizda po ceste + escape/relocalization podle LiDARu

pose = public_vars.estimated_pose;
path = public_vars.path;

% Kdyz neni odhad, zastav
if isempty(pose) || any(isnan(pose))
    public_vars.motion_vector = [0, 0];
    return
end

% espace a relocalization
if isfield(public_vars, "project") && isfield(public_vars.project, "mode")
    if strcmp(public_vars.project.mode, "escape")

        if ~isfield(read_only_vars, "lidar_distances") || isempty(read_only_vars.lidar_distances)
            public_vars.motion_vector = [0, 0];
            return
        end

        d = read_only_vars.lidar_distances(:);
        a = read_only_vars.lidar_config(:);

        n = min(numel(d), numel(a));
        d = d(1:n);
        a = a(1:n);

        valid = isfinite(d);
        if ~any(valid)
            public_vars.motion_vector = [0, 0];
            return
        end

        % "chodba / otevreny smer" = paprsek + sousedi
        score = -inf(n,1);
        for i = 1:n
            if ~valid(i)
                continue
            end

            im1 = i - 1;
            ip1 = i + 1;
            if im1 < 1, im1 = n; end
            if ip1 > n, ip1 = 1; end

            d0 = d(i);
            d1 = d(im1); if ~isfinite(d1), d1 = 0; end
            d2 = d(ip1); if ~isfinite(d2), d2 = 0; end

            score(i) = d0 + 0.5*d1 + 0.5*d2;
        end

        [~, idx] = max(score);

        best_rel_angle = a(idx);
        best_dist = d(idx);

        angle_error = atan2(sin(best_rel_angle), cos(best_rel_angle));

        % pri velkem uhlu nejdriv toc, pri malem uhlu jed dopredu
        max_vel = read_only_vars.agent_drive.max_vel;

        if abs(angle_error) > 0.7
            v_turn = 0.45;
            vR =  v_turn * sign(angle_error);
            vL = -v_turn * sign(angle_error);
        else
            v = min(0.5, 0.20 + 0.20 * min(best_dist, 1.5) / 1.5);
            k = 1.2;
            vR = v + k * angle_error;
            vL = v - k * angle_error;
        end

        vR = max(min(vR, max_vel), -max_vel);
        vL = max(min(vL, max_vel), -max_vel);

        public_vars.motion_vector = [vR, vL];

        if mod(read_only_vars.counter, 20) == 0
            disp("----- ESCAPE DEBUG -----")
            disp("pose:")
            disp(pose)
            disp("best_rel_angle:")
            disp(best_rel_angle)
            disp("best_dist:")
            disp(best_dist)
            disp("motion:")
            disp(public_vars.motion_vector)
        end

        return
    end
end

% Kdyz neni cesta, zastav
if isempty(path)
    public_vars.motion_vector = [0, 0];
    return
end

% LiDAR data
front = inf;
left = inf;
right = inf;
min_lidar = inf;

if isfield(read_only_vars, "lidar_distances") && ~isempty(read_only_vars.lidar_distances)
    vals = read_only_vars.lidar_distances(:);
    vals_valid = vals(isfinite(vals));

    if ~isempty(vals_valid)
        min_lidar = min(vals_valid);
    end

    if numel(vals) >= 8
        front = min([vals(1), vals(2), vals(8)]);
        left  = min([vals(2), vals(3), vals(4)]);
        right = min([vals(8), vals(7), vals(6)]);
    elseif ~isempty(vals_valid)
        front = min_lidar;
        left = min_lidar;
        right = min_lidar;
    end
end

% Opravdu kriticka situace -> lehce couvni
if min_lidar < 0.12
    public_vars.motion_vector = [-0.1, -0.1];
    return
end

% Target index
if ~isfield(public_vars, 'target_idx')
    public_vars.target_idx = 1;
end

public_vars.target_idx = min(public_vars.target_idx, size(path,1));
public_vars.target_idx = max(public_vars.target_idx, 1);

target = path(public_vars.target_idx,:);

dist = norm(target - pose(1:2));

if dist < 0.4
    if public_vars.target_idx < size(path,1)
        public_vars.target_idx = public_vars.target_idx + 1;
        target = path(public_vars.target_idx,:);
    end
end

% Rizeni na target
dx = target(1) - pose(1);
dy = target(2) - pose(2);
desired_theta = atan2(dy, dx);

angle_error = desired_theta - pose(3);
angle_error = atan2(sin(angle_error), cos(angle_error));

v_nominal = 0.8;

front_slow = 1.0;
front_critical = 0.20;

speed_scale = (front - front_critical) / (front_slow - front_critical);
speed_scale = max(0.20, min(speed_scale, 1.0));

turn_scale = max(0.35, cos(angle_error));

v = v_nominal * speed_scale * turn_scale;

left_safe = max(left, 0.10);
right_safe = max(right, 0.10);

k_wall = 0.15;
wall_bias = k_wall * ((1/right_safe) - (1/left_safe));

k = 1.5;
u = k * angle_error + wall_bias;

vR = v + u;
vL = v - u;

max_vel = read_only_vars.agent_drive.max_vel;
vR = max(min(vR, max_vel), -max_vel);
vL = max(min(vL, max_vel), -max_vel);

if abs(angle_error) > 0.8
    v_turn = 0.25;
    v_forward_small = 0.12;

    vR = v_forward_small + v_turn * sign(angle_error);
    vL = v_forward_small - v_turn * sign(angle_error);

    vR = max(min(vR, max_vel), -max_vel);
    vL = max(min(vL, max_vel), -max_vel);
end

public_vars.motion_vector = [vR, vL];

if mod(read_only_vars.counter, 20) == 0
    disp("---- CONTROL DEBUG ----------")
    disp("pose:")
    disp(pose)
    disp("path size:")
    disp(size(path,1))
    disp("target_idx:")
    disp(public_vars.target_idx)
    disp("target:")
    disp(target)
    disp("front / left / right / min:")
    disp([front left right min_lidar])
    disp("speed_scale:")
    disp(speed_scale)
    disp("wall_bias:")
    disp(wall_bias)
    disp("angle_error:")
    disp(angle_error)
    disp("motion:")
    disp(public_vars.motion_vector)
end

end