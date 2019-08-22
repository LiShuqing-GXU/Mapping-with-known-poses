%% Mapping with known poses
clear;
load('robot1.mat');
dt1=cell2mat(dt1);
% omometry_data1=cell2mat(omometry_data1);
X1=cell2mat(X1);
clearvars -except  dt1 omometry_data1 X1 scans1 
clc;
map_to_obsticale_vector;



%% discard k first entries because robot is static
k=40;
scans1(1:k)=[];
dt1(1:k)=[];
% omometry_data1(1:k,:)=[];
X1(1:k,:)=[];

%% initialize particles and maps
Num_Of_Sim_steps=length(dt1)-1; % dt1 is 327 time-steps before discarding k ellements
% Num_Of_Sim_steps=200; % length(dt1)-1:

CellNum_x=171;
CellNum_y=CellNum_x;

prob_map1=zeros(CellNum_x,CellNum_y,Num_Of_Sim_steps);
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
    robot_x=X1(t,1); robot_y=X1(t,2); robot_theta=X1(t,3);%+pi/2;%+3*pi/8+pi; %+X1(t+1,3);
    scans_cart = scan2cart(scans1{t,1}, robot_theta );
    robot_cell_pose = [round(CellNum_x/2+robot_x/res), round(CellNum_y/2+robot_y/res)];
    
    % Use only L measurements (uniformally sampled) to make simulation
    % faster
    L=500; %number of scans used from the available 1080
    scan_number=length(scans_cart); % the number of scans ( should be 1080)
    idx=randsample(scan_number,L);
    
    
    for j=1:length(idx)
        
        scan_cell=[robot_cell_pose(1)+round(scans_cart(idx(j),1)/res)...
                   robot_cell_pose(2)+round(scans_cart(idx(j),2)/res)];
        
        [~,~,~,X_cells,Y_cells]=bresenham1(map,...
                    [robot_cell_pose(1), robot_cell_pose(2) ;...
                    scan_cell(1), scan_cell(2)],0);
                
        % the robots cell needs to be the last one
        chek_firstx(j) =  (X_cells(1)==robot_cell_pose(2));
        chek_firsty(j) =  (Y_cells(1)==robot_cell_pose(1));
        if chek_firstx(j)==0
            X_cells=fliplr(X_cells);
            if chek_firsty(j)==0
                Y_cells=fliplr(Y_cells);
            end
            chek_firstx(j) =  (X_cells(1)==robot_cell_pose(2));
            chek_firsty(j) =  (Y_cells(1)==robot_cell_pose(1)); 
            
        elseif chek_firsty(j)==0
            Y_cells=fliplr(Y_cells);
            if chek_firstx(j)==0
                X_cells=fliplr(X_cells);
            end
            chek_firstx(j) =  (X_cells(1)==robot_cell_pose(2));
            chek_firsty(j) =  (Y_cells(1)==robot_cell_pose(1)); 
        end

         map=update_map(map,X_cells,Y_cells);

    end  
    check_first(:,1,t)=chek_firstx';
    check_first(:,2,t)=chek_firsty';
    
    
    prob_map1(:,:,t)=map;
    
    figure(1); clf;
    subplot(1,2,1)
    imagesc(map)
    colormap(flipud(gray))
    hold on
    scatter(robot_cell_pose(2),robot_cell_pose(1),'b');
    hold off
    axis square;
    camroll(90)
    drawnow 
    
    
    subplot(1,2,2)
    scatter(obsticle_vector(:,1),obsticle_vector(:,2), 1)
    hold on
    % plot real robot location
    plot_robot(X1(t,:)+[10 10 0],0.3)
    axis square;
    hold off
    
    
    pause(0.02)
    F(t)=getframe(gcf);
 
end

%% Save and Make Video
% video = VideoWriter(['Known_poses_mapping_',date,'.mp4']);
% video.FrameRate = 12;
% open(video)
% writeVideo(video,F);
% close(video)


