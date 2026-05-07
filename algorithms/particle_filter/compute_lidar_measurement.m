function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurement = zeros(1, length(lidar_config));

ray_origin = pose(1:2);
theta = pose(3);

for i= 1:length(lidar_config)
    direction = theta + lidar_config(i);

    intersections = ray_cast(ray_origin, map.walls, direction);

    if isempty(intersections)
        measurement(i)=inf;
    else
        dx = intersections(:,1) - ray_origin(1);
        dy = intersections(:,2) - ray_origin(2);
        d = sqrt(dx.^2 + dy.^2);
        measurement(i) = min(d);
    end
end



end

