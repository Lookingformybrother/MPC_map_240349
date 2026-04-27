function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here
%grid = read_only_vars.discrete_map.map;

raw_grid=read_only_vars.discrete_map.map;
step= read_only_vars.map.discretization_step;

clearance_m = 0.2;
inflate_radius_cells = ceil(clearance_m/step);

grid = inflate_map(raw_grid,inflate_radius_cells);



disp("unique grid values:")
disp(unique(grid(:))')

limits = read_only_vars.map.limits;
step = read_only_vars.map.discretization_step;
goal_xy = read_only_vars.map.goal(:)';


if isfield(public_vars, "estimated_pose") && ~isempty(public_vars.estimated_pose) && all(isfinite(public_vars.estimated_pose(1:2)))
    start_xy = public_vars.estimated_pose(1:2);
elseif isfield(read_only_vars, "mocap_pose")
    start_xy = read_only_vars.mocap_pose(1:2);
else
    path = [];
    return
end

[nRows, nCols] = size(grid);
start_rc = meters_to_grid(start_xy, limits, step, nRows, nCols);
goal_rc = meters_to_grid(goal_xy, limits, step, nRows, nCols);

start_rc = find_nearest_free(start_rc, grid);
goal_rc = find_nearest_free(goal_rc, grid);


gScore = inf(nRows, nCols);
fScore = inf(nRows, nCols);
openSet = false(nRows, nCols);
closedSet = false(nRows, nCols);

parentRow = zeros(nRows, nCols);
parentCol = zeros(nRows, nCols);

neighbors = [
    -1, 0,  1;
    1,  0,  1;
    0,  -1, 1;
    0,  1,  1;
    -1, -1, sqrt(2);
    -1, 1,  sqrt(2);
    1,  -1, sqrt(2);
    1,  1,  sqrt(2)
    ];

sr = start_rc(1);
sc=start_rc(2);
gr=goal_rc(1);
gc=goal_rc(2);

gScore(sr,sc)=0;
fScore(sr,sc)=heuristic(sr,sc,gr,gc);
openSet(sr,sc)=true;

found = false;

while any(openSet(:))
    [cr,cc] = get_best_open_node(openSet, fScore);
    if cr== gr && cc == gc
        found =true;
        break
    end
    openSet(cr,cc)=false;
    closedSet(cr,cc)=true;

    for k=1:size(neighbors,1)
        nr=cr+neighbors(k,1);
        nc=cc+neighbors(k,2);
        stepCost=neighbors(k,3);

        if nr<1 || nr>nRows || nc<1 || nc>nCols
            continue
        end
        if closedSet(nr,nc)
            continue
        end
        if is_occupied(grid,nr, nc)
            continue
        end

        if abs(neighbors(k,1)) == 1 && abs(neighbors(k,2)) == 1
            if is_occupied(grid, cr, nc) || is_occupied(grid,nr,cc)
                continue
            end
        end

        tentativeG = gScore(cr,cc)+stepCost;
        if ~openSet(nr,nc) || tentativeG <gScore(nr,nc)
            parentRow(nr,nc) = cr;
            parentCol(nr,nc)=cc;

            gScore(nr,nc) = tentativeG;
            fScore(nr,nc)=tentativeG+heuristic(nr,nc,gr,gc);
            openSet(nr,nc)=true;
        end
    end
end

if ~found
    path=[];
    return
end

grid_path = reconstruct_path(parentRow, parentCol, start_rc, goal_rc);

disp("found:")
disp(found)

disp("grid_path length:")
disp(size(grid_path,1))

path = zeros(size(grid_path,1),2);
for i = 1:size(grid_path,1)
    path(i,:)=grid_to_meters(grid_path(i,:), limits, step);
end

disp("start_xy:")
disp(start_xy)

disp("goal_xy:")
disp(goal_xy)

disp("start_rc:")
disp(start_rc)

disp("goal_rc:")
disp(goal_rc)

disp("grid at start:")
disp(grid(start_rc(1), start_rc(2)))

disp("grid at goal:")
disp(grid(goal_rc(1), goal_rc(2)))
end




