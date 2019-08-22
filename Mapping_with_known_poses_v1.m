%% Mapping with known poses
clear;
load('robot1.mat');
dt1=cell2mat(dt1);
omometry_data1=cell2mat(omometry_data1);
X1=cell2mat(X1);
clearvars -except  dt1 omometry_data1 X1 scans1 
clc;
%% discard k first entries because robot is static
k=40;
scans1(1:k)=[];
dt1(1:k)=[];
omometry_data1(1:k,:)=[];
X1(1:k,:)=[];

%% initialize particles and maps
% Num_Of_Sim_steps=length(dt1); % dt1 is 327 time-steps before discarding k ellements
Num_Of_Sim_steps=200; % length(dt1)-1:

CellNum_x=171;
CellNum_y=CellNum_x;

prob_map=zeros(CellNum_x,CellNum_y,Num_Of_Sim_steps);
map= OccGrid([CellNum_y,CellNum_x],1);

res=0.2; % 0.1 meters per cell [m/cell]



X_cells=[];
Y_cells=[];
robot_cell_pose=zeros(Num_Of_Sim_steps,2);

for t=1:Num_Of_Sim_steps
   
    disp('simulation progress [%]: ');
    sim_prog= t*100/Num_Of_Sim_steps;
    disp(sim_prog);

    
    % Get its state variables [x y theta]
    robot_x=X1(t,1); robot_y=X1(t,2); robot_theta=X1(t,3);
    scans_cart = scan2cart(scans1{t,1}, robot_theta );
    robot_cell_pose = [round(CellNum_x/2+robot_x/res), round(CellNum_y/2+robot_y/res)];
   
    for j=1:length(scans_cart)
        [~,~,~,X_cells,Y_cells]=bresenham1(map,...
                        [robot_cell_pose(1), robot_cell_pose(2) ;...
                        robot_cell_pose(1)+round(scans_cart(j,1)/res),...
                        robot_cell_pose(2)+round(scans_cart(j,2)/res)],0);

         map=update_map(map,X_cells,Y_cells);
    end  
   
    prob_map(:,:,t)=map;
    figure(1);clf;
    imagesc(ones(length(map))-map)    
    colormap gray   
    hold on
    scatter(robot_cell_pose(1),robot_cell_pose(2),'b');
    hold off
    drawnow 
 
end
%% Save and Make Video
% save the Best_Maps array
% save(['prob_map_',date],'prob_map')
% make video
% SLAM_video( map )


