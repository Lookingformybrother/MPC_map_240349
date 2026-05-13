function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE
% Odhad polohy z particle filteru.

estimated_pose = [NaN, NaN, NaN];

% 1) Primarne particle filter
if isfield(public_vars, "particles") && ~isempty(public_vars.particles)
    P = public_vars.particles;

    if size(P,2) >= 4
        w = P(:,4);

        valid = all(isfinite(P(:,1:4)), 2) & (w > 0);

        if isfield(public_vars, "project") && isfield(public_vars.project, "map_limits")
            lims = public_vars.project.map_limits;
            valid = valid & ...
                    P(:,1) >= lims(1) & P(:,1) <= lims(3) & ...
                    P(:,2) >= lims(2) & P(:,2) <= lims(4);
        end

        if any(valid)
            P = P(valid,:);
            w = P(:,4);

            s = sum(w);
            if s > 1e-12
                w = w / s;

                x = sum(w .* P(:,1));
                y = sum(w .* P(:,2));
                th = atan2(sum(w .* sin(P(:,3))), sum(w .* cos(P(:,3))));

                estimated_pose = [x, y, th];
                return
            end
        end
    end
end

% 2) Fallback na KF/EKF
if isfield(public_vars, "mu") && numel(public_vars.mu) >= 3
    if all(isfinite(public_vars.mu(1:3)))
        estimated_pose = public_vars.mu(:).';
        return
    end
end

end