function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

N = size(particle_measurements, 1);
weights = zeros(N,1);
sigma = 0.7; %zde si vytváříme proměnou pro porovnání lidaru


for i = 1:N
    pred=particle_measurements(i,:);
    real=lidar_distances;
    valid=isfinite(real) & isfinite(pred);

    if sum(valid)<3
        weights(i)=1e-12;
    else
        diff=pred(valid)-real(valid);
        mse=mean(diff.^2);
        if isnan(mse) || isinf(mse)
            weights(i)=1e-12;
        else
            weights(i)=exp(-mse/(2*sigma^2));
        end
    end
end


weights(~isfinite(weights))=1e-12;
weights=weights+1e-12;
weights=weights/sum(weights);


end

