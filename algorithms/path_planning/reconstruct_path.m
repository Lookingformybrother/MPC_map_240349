function grid_path = reconstruct_path(parentRow, parentCol, start_rc, goal_rc)
current =goal_rc;
grid_path=current;

while ~(current(1) == start_rc(1) && current(2) == start_rc(2))
    pr = parentRow(current(1), current(2));
    pc = parentCol(current(1), current(2));

    if pr == 0 && pc == 0
        grid_path = [];
        return
    end

    current = [pr,pc];
    grid_path = [current; grid_path];
end
end
