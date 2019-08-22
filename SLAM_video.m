function [  ] = SLAM_video( maps_over_time)

[~,~,time_steps]=size(maps_over_time);
for k = 1:time_steps
    % draw map
    figure(1); clf;   
    imagesc(ones(length(maps_over_time(:,:,k)))-maps_over_time(:,:,k)); 
    colormap gray;                            
    drawnow  
    pause(0.02)
    F(k)=getframe(gcf); 
end
%% Make Video
video = VideoWriter(['SLAM1_',date,'.mp4']);
video.FrameRate = 12;
open(video)
writeVideo(video,F);
close(video)

end

