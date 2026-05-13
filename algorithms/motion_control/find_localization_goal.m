function [goal_xy] = find_localization_goal(read_only_vars, public_vars)
%FIND_LOCALIZATION_GOAL Najde docasny cil pobliz nejblizsi prekazky

goal_xy = read_only_vars.map.goal(:)'; % fallback

if ~isfield(public_vars, "estimated_pose") || isempty(public_vars.estimated_pose) ...
        || any(isnan(public_vars.estimated_pose(1:2)))
    return
end

grid = read_only_vars.discrete_map.map;
limits = read_only_vars.map.limits;
step = read_only_vars.map.discretization_step;

[nRows, nCols] = size(grid);

start_xy = public_vars.estimated_pose(1:2);
start_rc = meters_to_grid(start_xy, limits, step, nRows, nCols);

% najdi vsechny obsazene bunky
occ = [];
for r = 1:nRows
    for c = 1:nCols
        if is_occupied(grid, r, c)
            occ = [occ; r c];
        end
    end
end

if isempty(occ)
    return
end

% nejblizsi prekazka
d2 = (occ(:,1) - start_rc(1)).^2 + (occ(:,2) - start_rc(2)).^2;
[~, idx] = min(d2);
wall_rc = occ(idx,:);

% chceme volnou bunku priblizne 0.6 m od prekazky
target_dist_cells = max(2, ceil(0.6 / step));

best_rc = [];
best_score = inf;

for r = max(1, wall_rc(1)-target_dist_cells-2):min(nRows, wall_rc(1)+target_dist_cells+2)
    for c = max(1, wall_rc(2)-target_dist_cells-2):min(nCols, wall_rc(2)+target_dist_cells+2)

        if is_occupied(grid, r, c)
            continue
        end

        d_wall = sqrt((r - wall_rc(1))^2 + (c - wall_rc(2))^2);
        d_start = sqrt((r - start_rc(1))^2 + (c - start_rc(2))^2);

        % chceme byt pobliz prekazky, ale ne v ni, a nechceme zbytecne daleko
        score = abs(d_wall - target_dist_cells) + 0.05 * d_start;

        if score < best_score
            best_score = score;
            best_rc = [r, c];
        end
    end
end

if ~isempty(best_rc)
    goal_xy = grid_to_meters(best_rc, limits, step);
end

end