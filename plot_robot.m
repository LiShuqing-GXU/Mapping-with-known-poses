       function plot_robot(pos,r)         
             mycolor = 'k';
             phi = linspace(0,2*pi,101);
%              r = 0.3; %robot radius
             % plot robot body
             plot(pos(1) + r*cos(phi),pos(2) + r*sin(phi),'Color',mycolor);
             hold on;
             % plot heading direction
             plot([pos(1), pos(1) + r*cos(pos(3))],[pos(2), pos(2) + r*sin(pos(3))],'Color',mycolor);      
             axis equal;    
                    
                    
       end  