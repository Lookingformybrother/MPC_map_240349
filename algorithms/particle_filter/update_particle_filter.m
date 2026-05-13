function [particles] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER
% PF s vyuzitim LiDARu + GNSS.
% Castice mimo mapu dostanou nulovou vahu.
% Kdyz je GNSS dostupne, pouzije se jako dalsi mereni pro vahy.
% Kdyz se filtr rozpadne, znovu se inicializuje okolo GNSS.

particles = public_vars.particles;

if isempty(particles)
    return
end

N = size(particles,1);

xmin = read_only_vars.map.limits(1);
ymin = read_only_vars.map.limits(2);
xmax = read_only_vars.map.limits(3);
ymax = read_only_vars.map.limits(4);

% GNSS dostupnost
gnss_available = false;
gnss = [NaN, NaN];

if isfield(read_only_vars, "gnss_position") && ~isempty(read_only_vars.gnss_position)
    if all(isfinite(read_only_vars.gnss_position(1:2)))
        gnss_available = true;
        gnss = read_only_vars.gnss_position(1:2);
    end
end

% GNSS covariance
Qgnss = [0.25 0; 0 0.25];   % fallback
if isfield(public_vars, "kf") && isfield(public_vars.kf, "Q")
    if all(size(public_vars.kf.Q) == [2 2]) && all(isfinite(public_vars.kf.Q), "all")
        Qgnss = public_vars.kf.Q;
    end
end

Qgnss = Qgnss + 1e-6*eye(2);

% Predikce
for i = 1:N
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
    particles(i,3) = atan2(sin(particles(i,3)), cos(particles(i,3)));
end

% Validni castice
in_map = particles(:,1) >= xmin & particles(:,1) <= xmax & ...
         particles(:,2) >= ymin & particles(:,2) <= ymax & ...
         all(isfinite(particles(:,1:3)), 2);

valid_idx = find(in_map);

lidar_weights = zeros(N,1);
gnss_weights  = ones(N,1);   % neutralni, kdyz GNSS neni

% lidar korekce
if ~isempty(valid_idx)
    measurements = zeros(numel(valid_idx), length(read_only_vars.lidar_config));

    for k = 1:numel(valid_idx)
        i = valid_idx(k);
        measurements(k,:) = compute_lidar_measurement( ...
            read_only_vars.map, particles(i,1:3), read_only_vars.lidar_config);
    end

    lidar_weights(valid_idx) = weight_particles(measurements, read_only_vars.lidar_distances);
end

% GNSS korekce
if gnss_available && ~isempty(valid_idx)
    gnss_weights(valid_idx) = compute_gnss_weights( ...
        particles(valid_idx,1:2), gnss, Qgnss);
end

% Spojeni vah
weights = zeros(N,1);

% jak moc ma GNSS pomahat
gnss_gain = 1.2;   % 0.8 az 2.0, vetsi = vetsi vliv GNSS

if sum(lidar_weights(valid_idx)) > 1e-12
    % kombinace lidar * GNSS
    weights(valid_idx) = lidar_weights(valid_idx) .* (gnss_weights(valid_idx).^gnss_gain);
elseif gnss_available
    % kdyz by se lidar rozpadl, pouzij aspon GNSS
    weights(valid_idx) = gnss_weights(valid_idx);
end

% mimo mapu = nulova vaha
weights(~isfinite(weights)) = 0;
weights(weights < 0) = 0;

s = sum(weights);

% Kdyz se vse rozpadne -> reinicializace
if s <= 1e-12
    particles = reinitialize_particles(N, xmin, ymin, xmax, ymax, gnss_available, gnss, Qgnss);
    return
end

weights = weights / s;
particles(:,4) = weights;

% Resampling
Neff = 1 / sum(weights.^2);

if Neff < 0.8 * N
    particles = resample_particles(particles, weights);

    % castice mimo mapu po resamplingu nahrad
    bad = particles(:,1) < xmin | particles(:,1) > xmax | ...
          particles(:,2) < ymin | particles(:,2) > ymax | ...
          any(~isfinite(particles(:,1:3)), 2);

    if any(bad)
        particles(bad,:) = reinitialize_particles(sum(bad), xmin, ymin, xmax, ymax, gnss_available, gnss, Qgnss);
    end

    particles(:,4) = 1/N;
end

end


function w = compute_gnss_weights(particles_xy, gnss, Q)
% Vypocet vah podle 2D gaussovskeho modelu GNSS

err = particles_xy - gnss(:).';
invQ = inv(Q);

mahal2 = sum((err * invQ) .* err, 2);
w = exp(-0.5 * mahal2);

w(~isfinite(w)) = 0;
w(w < 0) = 0;

s = sum(w);
if s > 1e-12
    w = w / s;
else
    w = ones(size(w)) / numel(w);
end
end


function particles = reinitialize_particles(N, xmin, ymin, xmax, ymax, gnss_available, gnss, Qgnss)
% Kdyz je GNSS dostupne, inicializuj okolo GNSS, jinak po cele mape.

particles = zeros(N,4);

if gnss_available
    sigma_x = max(0.5, sqrt(Qgnss(1,1)));
    sigma_y = max(0.5, sqrt(Qgnss(2,2)));

    particles(:,1) = gnss(1) + sigma_x * randn(N,1);
    particles(:,2) = gnss(2) + sigma_y * randn(N,1);

    particles(:,1) = max(xmin, min(xmax, particles(:,1)));
    particles(:,2) = max(ymin, min(ymax, particles(:,2)));
else
    particles(:,1) = xmin + (xmax - xmin) * rand(N,1);
    particles(:,2) = ymin + (ymax - ymin) * rand(N,1);
end

particles(:,3) = -pi + 2*pi*rand(N,1);
particles(:,4) = 1 / N;

end