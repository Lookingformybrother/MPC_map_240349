function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER
% Kdyz je dostupne GNSS, inicializuj castice kolem GNSS.
% Jinak rovnomerne po cele mape.

N = 600; 

xmin = read_only_vars.map.limits(1);
ymin = read_only_vars.map.limits(2);
xmax = read_only_vars.map.limits(3);
ymax = read_only_vars.map.limits(4);

particles = zeros(N,4);

gnss_available = false;
gnss = [NaN, NaN];

if isfield(read_only_vars, "gnss_position") && ~isempty(read_only_vars.gnss_position)
    if all(isfinite(read_only_vars.gnss_position(1:2)))
        gnss_available = true;
        gnss = read_only_vars.gnss_position(1:2);
    end
end

if gnss_available
    % siroke rozhozeni kolem GNSS, at filtr neni prehnane sebevedomy hned na zacatku
    sigma_x = 1.0;
    sigma_y = 1.0;

    particles(:,1) = gnss(1) + sigma_x * randn(N,1);
    particles(:,2) = gnss(2) + sigma_y * randn(N,1);

    % clamp do mapy
    particles(:,1) = max(xmin, min(xmax, particles(:,1)));
    particles(:,2) = max(ymin, min(ymax, particles(:,2)));
else
    % bez GNSS rozmisteni po cele mape
    particles(:,1) = xmin + (xmax - xmin) * rand(N,1);
    particles(:,2) = ymin + (ymax - ymin) * rand(N,1);
end

particles(:,3) = -pi + 2*pi*rand(N,1);
particles(:,4) = 1/N;

public_vars.particles = particles;

end