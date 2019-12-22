classdef utils
    % utilities methods
    properties
        % null        
    end
    methods (Static)
        function [relative] = getPoseInRCM(vs2rcm,ee_pose)
            % Knowing relative position of RCM frame and Vision Sensor frame
            % this method returns a pose in RCM frame, given a pose in VisionSensor Frame.
            %
            % vs2rcm : 6x1 vector of position and orientation of RCM wrt VS
            % ee_pose : pose of EE wrt VS
            %
            % extracting rot. matrix associated to orientation described in euler
            % angles of rcm wrt vision sensor.
            % This is used in calculating the relative position
            rotm_VS_RCM = eul2rotm(vs2rcm(4:6)', 'XYZ'); % vrep default eul represent.
            
            % This rot. matrix is the one attached to the relative position of EE wrt
            % Vision Sensor.
            % ee_pose(4:6) is an euler angles representation from which i get a rot.
            % matrix. This is used in calculating the new orientation
            rotm_VS_EE = eul2rotm(ee_pose(4:6)',  'XYZ');
            
            % This is [x y z] expressed in RCM frame starting from [x y z] in VS reference.
            relative_position = -(rotm_VS_RCM')*vs2rcm(1:3) + (rotm_VS_RCM')*ee_pose(1:3);
            
            % This R4 matrix is the rotation matrix from RCM to EE
            % Is the result of concatenating RCM -> VS -> EE matrices
            R4 = rotm_VS_RCM\rotm_VS_EE; % inv(RotMatrix)*R3 - same notation
            
            % From this rotation matrix we extract euler angles orientation
            relative_orientation = rotm2eul(R4,'XYZ'); % to be solved
            
            % output
            relative = [relative_position; relative_orientation'];
            
        end
                
        function [error] = computeError(desired, current)
            % computes error between poses
            error = [desired(1:3) - current(1:3); angdiff(current(4:6), desired(4:6))];            
        end
              
    end
end