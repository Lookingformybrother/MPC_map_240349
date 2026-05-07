function occ = is_occupied(grid,r,c)
val = grid(r,c);
occ = ~isfinite(val)|| (val~=0);
end
