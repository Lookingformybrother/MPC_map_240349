function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% %%tvorba mapy:
% 9,9 %cil robota
% 0,0,20,20 %hranice mapy [x_min, y_min, x_max, y_max]
% 0, 0, 20, 0 % stěny ve formátu: [x1, y1, x2, y2]
% 20, 0, 20, 20
% 20, 20, 0, 20
% 0, 20, 0, 0
% 1, 1, 5, 1
% inf %všechno nad inf jsou stěny, všechno pod jsou oblasti
% 0,0,20,0,20,20,0,20 %asi gnss denied oblast [x1,y1,x2,y2,x3,y3,x4,y4]
% %%protože do texťáku nejdou přidávat poznámky, tak si to odložím sem.





% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(read_only_vars,public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);



end

