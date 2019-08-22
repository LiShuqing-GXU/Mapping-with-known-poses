function [  ] = SLAM_video2( maps_over_time,maps_cell)

[~,~,time_steps]=size(maps_over_time);
for k = 1:time_steps
    % draw map
    figure(1); clf;   
    imagesc(ones(length(maps_over_time(:,:,k)))-maps_over_time(:,:,k)); 
    colormap gray;  
    hold on
    scatter(maps_cell(k,1),maps_cell(k,2),'b');
    hold off
    drawnow  
    pause(0.02)
    F(k)=getframe(gcf); 
end
%% Make Video
video = VideoWriter(['SLAM1_',date,'.mp4']);
video.FrameRate = 8;
open(video)
writeVideo(video,F);
close(video)

end

