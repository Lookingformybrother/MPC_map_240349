function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here

N= size(particles,1);
new_particles=zeros(size(particles));

c=cumsum(weights);
r=rand/N;

i=1;
for m=1:N
    u= r+(m-1)/N;
    while u>c(i)
        i=i+1;
    end
    new_particles(m,:)=particles(i,:);
end

new_particles(:,4) = 1/N;

% maly jitter
new_particles(:,1) = new_particles(:,1) + 0.02*randn(size(new_particles,1),1);
new_particles(:,2) = new_particles(:,2) + 0.02*randn(size(new_particles,1),1);
new_particles(:,3) = new_particles(:,3) + 0.01*randn(size(new_particles,1),1);

for i = 1:size(new_particles,1)
    new_particles(i,3) = atan2(sin(new_particles(i,3)), cos(new_particles(i,3)));
end


end

