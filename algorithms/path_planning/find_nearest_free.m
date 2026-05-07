function rc_free = find_nearest_free(rc, grid)
r0 = rc(1);
c0 = rc(2);

[nRows, nCols] = size(grid);

if ~is_occupied(grid,r0,c0)
    rc_free=rc;
    return
end

maxRadius= max(nRows,nCols);

for rad=1:maxRadius
    rMin=max(1,r0-rad);
    rMax=min(nRows,r0+rad);
    cMin=max(1,c0-rad);
    cMax=min(nCols, c0+rad);

    for r=rMin:rMax
        for c=cMin:cMax
            if ~is_occupied(grid,r,c)
                rc_free = [r,c];
                return
            end
        end
    end
end

rc_free=[];
end
