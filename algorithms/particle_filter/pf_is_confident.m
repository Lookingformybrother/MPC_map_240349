function [is_confident, stats] = pf_is_confident(public_vars)

is_confident = false;
stats = [inf, inf, inf];

if ~isfield(public_vars, "particles") || isempty(public_vars.particles)
    return
end

P=public_vars.particles;
valid=all(isfinite(P(:,1:3)),2);
P=P(valid,:);

% vahy
if size(P,2) >= 4
    w = P(:,4);
    w(~isfinite(w)) = 0;

    if sum(w) > 0
        w = w / sum(w);
    else
        w = ones(size(P,1),1) / size(P,1);
    end
else
    w = ones(size(P,1),1) / size(P,1);
end

mx = sum(P(:,1) .* w);
my = sum(P(:,2) .* w);

sx = sqrt(sum(w .* (P(:,1) - mx).^2));
sy = sqrt(sum(w .* (P(:,2) - my).^2));

% kruhova "odchylka" uhlu
c = sum(cos(P(:,3)) .* w);
s = sum(sin(P(:,3)) .* w);
R = sqrt(c^2 + s^2);

if R < 1e-6
    stheta = pi;
else
    stheta = sqrt(max(0, -2 * log(R)));
end

stats = [sx, sy, stheta];

% jednoducha proxy pro "jeden shluk"
is_confident = (sx < 0.5) && (sy < 0.5) && (stheta < 0.7);

end