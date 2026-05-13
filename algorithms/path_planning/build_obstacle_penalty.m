function [penalty_map] = build_obstacle_penalty(grid, radius_cells, penalty_gain)
%BUILD_OBSTACLE_PENALTY
% Vytvori soft cost map podle vzdalenosti od prekazek.
% Blizko zdi = vyssi cena, daleko od zdi = nizka nebo nulova cena.

[nRows, nCols] = size(grid);
penalty_map = zeros(nRows, nCols);

% seznam vsech obsazenych bunek
occupied_cells = [];

for r = 1:nRows
    for c = 1:nCols
        if is_occupied(grid, r, c)
            occupied_cells = [occupied_cells; r, c];
        end
    end
end

if isempty(occupied_cells)
    return
end

for r = 1:nRows
    for c = 1:nCols

        % prekazka sama o sobe ma nekonecnou cenu
        if is_occupied(grid, r, c)
            penalty_map(r, c) = inf;
            continue
        end

        % vzdalenost k nejblizsi prekazce
        d2 = (occupied_cells(:,1) - r).^2 + (occupied_cells(:,2) - c).^2;
        d = sqrt(min(d2));

        % jen v okoli prekazek pridej penalty
        if d < radius_cells
            q = (radius_cells - d) / radius_cells;
            penalty_map(r, c) = penalty_gain * q^2;
        else
            penalty_map(r, c) = 0;
        end
    end
end

end