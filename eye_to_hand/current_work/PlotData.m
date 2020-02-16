classdef PlotData
    properties
    end
    methods (Static)
        function [] = plot_EE(spot,x,y,z)
            
            figure(spot);
            grid on;
            
            
            if(spot==1) color = 'r';
            elseif spot==2 color = 'b';
            elseif spot==3 color = 'k';
            elseif spot==4 color = 'g';
            elseif spot==5 color = 'm';
            elseif spot==6 color = 'y';
            end
            
            scatter3( x,y,z,'.',color)
            
            xlabel('x');
            ylabel('y');
            zlabel('z');
            title(['EE position toward spot n. ',num2str(spot)]);
            
        end
        
        function [] = plot_ForceAndImageErr(spot, force, total_error)
            
            figure(spot);
            
            subplot(2,1,1);
            grid on
            plot(force,'.');
            ylabel('norm force');
            ylim([min(force) max(force)]);
            
            xlabel('time');
            xlim([0 length(force)]);
            title(['norm of force correction toward spot n.',num2str(spot)]);
            
            subplot(2,1,2);
            grid on
            plot(total_error,'.');
            ylabel('image error');
            ylim([min(total_error) max(total_error)]);
            xlabel('time');
            xlim([0 length(force)]);
            
            title(['norm of total error toward spot n.',num2str(spot)]);
            
            
        end
        
    end
    
end

