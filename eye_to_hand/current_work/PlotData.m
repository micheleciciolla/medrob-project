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
            % they are not used but still useful for future work
            figure(spot);
            
            subplot(2,1,1);
            grid on
            plot(force,'.');
            ylabel('norm force');
            ylim([min(force) max(force)]);
            
            xlabel('time');
            xlim([0 length(force)]);
            title(['norm of force correction toward spot n.', num2str(spot)]);
            
            subplot(2,1,2);
            grid on
            plot(total_error,'.');
            ylabel('image error');
            ylim([min(total_error) max(total_error)]);
            xlabel('time');
            xlim([0 length(force)]);
            
            title(['norm of total error toward spot n.', num2str(spot)]);
            
            
        end
        
        function [] = plot_image_error_and_force(spot, u_ee, v_ee, u_desired, v_desired, force, time)
            
            % figure(spot);
            
            subplot(2,1,1)
            
            % plotting current position
            plot = scatter( [u_ee(1), u_ee(2), u_ee(3), u_ee(4)],...
                [v_ee(1), v_ee(2), v_ee(3), v_ee(4)], 'o','b');
            
            % plotting desired position
            plot = scatter( [u_desired(1,spot), u_desired(2,spot), u_desired(3,spot), u_desired(4,spot) ],...
                [v_desired(1,spot), v_desired(2,spot), v_desired(3,spot), v_desired(4,spot)], 'r', 'o','filled');
            
            hold on
            grid on
            title(['image error convergence to spot n. ',num2str(spot)])
            
            
            subplot(2,1,2)
            
            color = 'r';
            if force == 0 color = 'k';
            end
            stem(time, force,color,'filled', 'LineWidth', 2);
            hold on
            xlabel("time"); ylabel("force norm"); title("FORCE SENSOR PLOT");
        end
        
    end
    
end

