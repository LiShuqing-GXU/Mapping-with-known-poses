%% Particle Filter implementation

%% Load Robot 1 data
load('robot1.mat');
dt1=cell2mat(dt1);
omometry_data1=cell2mat(omometry_data1);
X1=cell2mat(X1);
%% Load Robot 2 data
load('robot2.mat');
dt2=cell2mat(dt2);
omometry_data2=cell2mat(omometry_data2);
X2=cell2mat(X2);
%%
clearvars -except  dt1 omometry_data1 X1 scans1 dt2 omometry_data2 X2 scans2
clc;


%% initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%map%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
map_to_obsticale_vector;
% map_ranges is the vector [min(x) max(x) min(y) max(y)] 
map_ranges = [min(obsticle_vector(:,1)) max(obsticle_vector(:,1)) ...
              min(obsticle_vector(:,2)) max(obsticle_vector(:,2))]; 
          
xmin=map_ranges(1);  xmax=map_ranges(2); ymin=map_ranges(3);  ymax=map_ranges(4);         

   
NumOfMix = 1000; % the number of ellements from the map we will take randomly and make a GMM from
gm=GMM_map( obsticle_vector(randsample(2629,NumOfMix),:) ,map_ranges,0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% initialize particles
N = 500; % numbre of particles

       

% generate particles uniformly distributed - state is (X,Y,Theta)_t
particles1 = [xmin - (xmin-xmax)*rand(N,1), ymin - (ymin-ymax)*rand(N,1), 2*pi*rand(N,1)];
% particles2 cant do global localization because robot 2 is in a corridor,
% therefor, the readings fit to another corridor that is in the map, and we
% need to initialize the particles in the vacinity of the true position, we
% can try "particles2 = [xmin - (xmin-xmax)*rand(N,1), ymin - (ymin-ymax)*rand(N,1), 2*pi*rand(N,1)]"
% and see it doesnt work.
particles2 = [(X2(1,1)+10)*ones(N,1), (X2(1,2)+10)*ones(N,1),zeros(N,1)]+normrnd(0,1,[N,3]);
%%uncomment to show
scatter(obsticle_vector(:,1),obsticle_vector(:,2))
hold on
scatter(particles1(:,1),particles1(:,2),'.')
scatter(particles2(:,1),particles2(:,2),'.')

%% localize
estimate1 = zeros(length(dt1),3); 
particles1_for_Plot=zeros(N,3,length(dt1));
estimate2 = zeros(length(dt1),3); 
particles2_for_Plot=zeros(N,3,length(dt1));
for k = 1:length(dt2) % dt2 is shorter 
    
    % estimate based on equally weighted particals
    estimate1(k,:)=mean(particles1,1);
    estimate2(k,:)=mean(particles2,1);

    % draw map
    clf;
    scatter(obsticle_vector(:,1),obsticle_vector(:,2)); hold on;
    % plot real robot location
    plot_robot(X1(k,:)+[10 10 0],0.3)
    plot_robot(X2(k,:)+[10 10 0],0.3)

    % plot 30 of robot's LIDAR readings
    plot_observations( X1(k,:)+[10 10 0], scans1{k,1})
    plot_observations( X2(k,:)+[10 10 0], scans2{k,1})

    % plot the estimated robot location twice the radius of the actual
    % robot
    plot_robot(estimate1(k,:),0.6)
    plot_robot(estimate2(k,:),0.6)


    % move partical set according to differential drive model and odometry
    % readings + white noise
    particles1= move_prtcls( particles1,omometry_data1(k,1), omometry_data1(k,2),dt1(k) );
    % move_prtcls2 allows you to controll the process noise std's 
    particles2= move_prtcls( particles2,omometry_data2(k,1), omometry_data2(k,2),dt2(k));

    % plot the particals
    scatter(particles1(:,1),particles1(:,2),'.')
    scatter(particles2(:,1),particles2(:,2),'.')

    pause(0.02)
    
    % assign weight to each partical 
    particles1_weight  = weigh_particles( particles1, gm, scans1{k,1} );
    particles2_weight  = weigh_particles( particles2, gm, scans2{k,1} );

    % re-sample according to weights
    particles1 = particles1(randsample((1:N),N,true,particles1_weight),:);
    particles2 = particles2(randsample((1:N),N,true,particles2_weight),:);


    disp('time step: ')
    disp(k)
    
    % ploting array
    particles1_for_Plot(:,:,k)=particles1;
    particles2_for_Plot(:,:,k)=particles2;

    
end


%% plot data

for k = 1:length(dt2)
    % draw map
    figure(2); clf;

    scatter(obsticle_vector(:,1),obsticle_vector(:,2)); 
    axis([xmin xmax ymin ymax ])
    hold on;
    % plot real robot location
    plot_robot(X1(k,:)+[10 10 0],0.3)
    plot_robot(X2(k,:)+[10 10 0],0.3)
    % plot 30 of robot's LIDAR readings
    plot_observations( X1(k,:)+[10 10 0], scans1{k,1})
    plot_observations( X2(k,:)+[10 10 0], scans2{k,1})
    % plot the estimated robot location twice the radius of the actual
    % robot
    plot_robot(estimate1(k,:),0.6)
    plot_robot(estimate2(k,:),0.6)
    % plot the particals
    scatter(particles1_for_Plot(:,1,k),particles1_for_Plot(:,2,k),'.')
    scatter(particles2_for_Plot(:,1,k),particles2_for_Plot(:,2,k),'.')
    
    drawnow  
    F(k)=getframe(gcf);
    
    pause(dt2(k))
    

end
%% Make Video
% video = VideoWriter('robot1&2.mp4');
% video.FrameRate = 12;
% open(video)
% writeVideo(video,F);
% close(video)

































