%% Mapping with known poses

clear;

load('robot2.mat');
dt2=cell2mat(dt2);
X2=cell2mat(X2);

load('robot1.mat');
dt1=cell2mat(dt1);
X1=cell2mat(X1);

clearvars -except  dt2 X2 scans2 dt1 X1 scans1 

clc;
map_to_obsticale_vector;



%% discard k first entries because robot is static
k=40;
scans2(1:k)=[];
dt2(1:k)=[];
X2(1:k,:)=[];
scans1(1:k)=[];
dt1(1:k)=[];
X1(1:k,:)=[];

%% initialize particles and maps
Num_Of_Sim_steps=length(dt2)-1; % dt1 is 327 time-steps before discarding k ellements
% Num_Of_Sim_steps=200; % length(dt1)-1:

CellNum_x=171;
CellNum_y=CellNum_x;

prob_map=zeros(CellNum_x,CellNum_y,Num_Of_Sim_steps);

fused_map= OccGrid([CellNum_y,CellNum_x],1);
map1=fused_map;
map2=fused_map;


res=0.2; % 0.1 meters per cell [m/cell]



X_cells=[];
Y_cells=[];

robot1_cell_pose=zeros(Num_Of_Sim_steps,2);



for t=1:Num_Of_Sim_steps
   
    disp('simulation progress [%]: ');
    sim_prog= t*100/Num_Of_Sim_steps;
    disp(sim_prog);
    
   
    
    % Get robot 1's state variables [x y theta]
    robot1_x=X1(t,1); robot1_y=X1(t,2); robot1_theta=X1(t,3);
    scans_cart1 = scan2cart(scans1{t,1}, robot1_theta );
    robot1_cell_pose = [40+round(CellNum_x/2+robot1_x/res), 20+round(CellNum_y/2+robot1_y/res)];
    
    % Use only L measurements (uniformally sampled) to make simulation
    % faster
    L=100; %number of scans used from the available 1080
    scan_number1=length(scans_cart1); % the number of scans ( should be 1080)
    idx=randsample(scan_number1,L);
    
    
    for j=1:length(idx)
        scan_cell=[robot1_cell_pose(1)+round(scans_cart1(idx(j),1)/res)...
                   robot1_cell_pose(2)+round(scans_cart1(idx(j),2)/res)];
        
        [~,~,~,X_cells,Y_cells]=bresenham1(fused_map,...
                    [robot1_cell_pose(1), robot1_cell_pose(2) ;...
                    scan_cell(1), scan_cell(2)],0);
                
        % the robots cell needs to be the last one
        chek_firstx(j) =  (X_cells(1)==robot1_cell_pose(2));
        chek_firsty(j) =  (Y_cells(1)==robot1_cell_pose(1));
        if chek_firstx(j)==0
            X_cells=fliplr(X_cells);
            if chek_firsty(j)==0
                Y_cells=fliplr(Y_cells);
            end
            chek_firstx(j) =  (X_cells(1)==robot1_cell_pose(2));
            chek_firsty(j) =  (Y_cells(1)==robot1_cell_pose(1)); 
            
        elseif chek_firsty(j)==0
            Y_cells=fliplr(Y_cells);
            if chek_firstx(j)==0
                X_cells=fliplr(X_cells);
            end
            chek_firstx(j) =  (X_cells(1)==robot1_cell_pose(2));
            chek_firsty(j) =  (Y_cells(1)==robot1_cell_pose(1)); 
        end
%          map1=update_map(fused_map,X_cells,Y_cells);

         fused_map=update_map(fused_map,X_cells,Y_cells);

    end  

    
    
   
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    
    % Get robot 2's state variables [x y theta]
    robot2_x=X2(t,1); robot2_y=X2(t,2); robot2_theta=X2(t,3);%+pi/2;%+3*pi/8+pi; %+X1(t+1,3);
    scans_cart2 = scan2cart(scans2{t,1}, robot2_theta );
    robot2_cell_pose = [40+round(CellNum_x/2+robot2_x/res),20+ round(CellNum_y/2+robot2_y/res)];
    
    % Use only L measurements (uniformally sampled) to make simulation
    % faster
    scan_number2=length(scans_cart2); % the number of scans ( should be 1080)
    idx=randsample(scan_number2,L);
    
    for j=1:length(idx)
        scan_cell=[robot2_cell_pose(1)+round(scans_cart2(idx(j),1)/res)...
                   robot2_cell_pose(2)+round(scans_cart2(idx(j),2)/res)];
        
        [~,~,~,X_cells,Y_cells]=bresenham1(fused_map,...
                    [robot2_cell_pose(1), robot2_cell_pose(2) ;...
                    scan_cell(1), scan_cell(2)],0);
                
        % the robots cell needs to be the last one
        chek_firstx(j) =  (X_cells(1)==robot2_cell_pose(2));
        chek_firsty(j) =  (Y_cells(1)==robot2_cell_pose(1));
        if chek_firstx(j)==0
            X_cells=fliplr(X_cells);
            if chek_firsty(j)==0
                Y_cells=fliplr(Y_cells);
            end
            chek_firstx(j) =  (X_cells(1)==robot2_cell_pose(2));
            chek_firsty(j) =  (Y_cells(1)==robot2_cell_pose(1)); 
            
        elseif chek_firsty(j)==0
            Y_cells=fliplr(Y_cells);
            if chek_firstx(j)==0
                X_cells=fliplr(X_cells);
            end
            chek_firstx(j) =  (X_cells(1)==robot2_cell_pose(2));
            chek_firsty(j) =  (Y_cells(1)==robot2_cell_pose(1)); 
        end
%          map2=update_map(fused_map,X_cells,Y_cells);

         fused_map=update_map(fused_map,X_cells,Y_cells);

    end 
    
%     fused_map2=0.5*map1+0.5*map2;
%     fused_map=(map1.^0.5).*(map2.^0.5);
%     fused_map=fused_map/max(max(fused_map));
    
     prob_map(:,:,t)=fused_map;
     
     
    %% Plotting
    figure(1); clf;
    subplot(1,2,1)
    imagesc(fused_map)
    colormap(flipud(gray))
    hold on
    scatter(robot1_cell_pose(2),robot1_cell_pose(1),'b');
    scatter(robot2_cell_pose(2),robot2_cell_pose(1),'b');
    hold off
    axis square;
    camroll(90)
    drawnow 
    
    
    subplot(1,2,2)
    scatter(obsticle_vector(:,1),obsticle_vector(:,2), 1)
    hold on
    % plot real robot location
    plot_robot(X1(t,:)+[10 10 0],0.3)
    plot_robot(X2(t,:)+[10 10 0],0.3)
    axis square;
    hold off
    
    
    pause(0.02)
    F(t)=getframe(gcf);
 
end
%% Save and Make Video
video = VideoWriter(['robots1&2_globalMap_',date,'.mp4']);
video.FrameRate = 12;
open(video)
writeVideo(video,F);
close(video)
