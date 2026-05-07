function inflated = inflate_map(grid,radius_cells)
[nRows, nCols]= size(grid);

inflated=grid;
occupied = false(nRows,nCols);

for r=1:nRows
    for c=1:nCols
        if is_occupied(grid,r,c)
            occupied(r,c) = true;
        end
    end
end

for r=1:nRows
    for c=1:nCols
        if occupied(r,c)
            rMin = max(1,r-radius_cells);
            rMax = min(nRows, r+radius_cells);
            cMin = max(1, c-radius_cells);
            cMax = min(nCols, c+radius_cells);

            for rr = rMin:rMax
                for cc = cMin:cMax
                    inflated(rr,cc)=1;
                end
            end
        end
    end
end
end

         