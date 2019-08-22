function [ particles_state,particles_weight ] = move_weigh( obstacle_vector, particles_state, scan_cell, omometry_data1,dt )
%move_weigh moves the particle's sent state according to the known motion
%model, and subsequently weigh each particle accrding to the likelyhood
%function built from the particle's obsitcale vctor. This function receives
%one particle's sate only (as arow vector), and corresponding obsitcale
%vector. Aswell as odomentry data and time interval.
%   obstacle_vector is a vector of Num_of_obsticales-by-2 of x and y
%   coordinates of each obsticale in the coresponding particle's map.
%   omometry_data1 is the linear and angular velocity at this time ste, and
%   dt1 is the time interval we use.


%% move particles
% we use the same function we used in the localization algorithms, but
% instead of Np particles sent before, particles_state only contains the
% state of one particle in this simulation.
particles_state= move_prtcls( particles_state,omometry_data1(1), omometry_data1(2),dt );
%% Build a likelyhood function from
% empirical cov-matrix, can be changed with trial and error.
sigma=[0.8 0; 0 0.8]; 
gm = gmdistribution(obstacle_vector(:,:),sigma);

% As done in the localization simulations, the obticale vector of the
% particle is turned into a GMM with the simple and unnecessary function of
% gm=GMM_map( obstacle_vector(:,:) ,[0,0,0,0],0), however we will use it
% only if we want to plot the GMM as below. this can be equivalently done
% by the above (exacly as GMM_map does)
%%%%%%%%%%%%%%%%%%%%%UNCOMMENT TO PLOT LIKLEYHOOD%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Extracting the map limits is aonly needed for the plot of the GMM (the
% % likelyhood). If we dont want to plot it we pass '0' insted od '1' at
% the % last variable in GMM_map(). however we still need to pass something
% to map_ranges % even if we dont want to plot. in that case we can just
% send '0' or % something. for plotting use: gm=GMM_map(
% obstacle_vector(:,:) ,map_ranges,1);

% map_ranges = [min(obstacle_vector(:,1)) max(obstacle_vector(:,1)) ...
%               min(obstacle_vector(:,2)) max(obstacle_vector(:,2))];
% xmin=map_ranges(1);  xmax=map_ranges(2); ymin=map_ranges(3);
% ymax=map_ranges(4); gm=GMM_map( obstacle_vector(:,:) ,map_ranges,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% weigh the particle
% weigh_particles_slam is a slight modification of the procedure used in
% the PF localization simulation, and thats why its not so elegant. We
% weigh the particle by attaching the ray scans to it. The end-point
% coordinate of each ray is evaluated using the likelyhood seen above, and
% returned in log values for reasons explained in the function.
particles_weight  = weigh_particles_slam( particles_state, gm, scan_cell );
end

