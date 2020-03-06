classdef kinematicsECM
    % from distributed/dVKinematics.cpp 
    % (Marco Ferro's file)
    properties
        t = zeros(7); % data structure to save data
        J = zeros(3,4); % jacobian
    end
    methods (Static)
        
            function [pos] = direct_kinematics(Q)
            
            R = zeros(4,4);
            Ret = zeros(3,3);
            p = zeros(1,3);
            % computes position of EE wrt ECM given joints values
            % output is 3x1 vector
                       
            q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);
             
            t(2) = cos(q1);
            t(3) = sin(q4);
            t(4) = cos(q4);
            t(5) = sin(q1);
            t(6) = sin(q2);
            t(7) = cos(q2);
            

            R(1,1) = t2*t4 - t3*t5*t6;
            R(1,0) = -t2*t3 - t4*t5*t6;
            R(1,3) = -t5*t7;

            p(1) = -q3*t5*t7;
            R(2, 1) = t4*t5 + t2*t3*t6;
            R(2, 2) = -t3*t5 + t2*t4*t6;
            R(2, 3) = t2*t7;
            p(2) = q3*t2*t7;
            R(3, 1) = -t3*t7;
            R(3, 2) = -t4*t7;
            R(3, 3) = t6;
            p(3) = q3*t6;

            Ret(1,1) = R(1,1);
            Ret(1,2) = R(1,2);
            Ret(1,3) = R(1,3);
            Ret(2,1) = R(2,1);
            Ret(2,2) = R(2,2);
            Ret(2,3) = R(2,3);
            Ret(3,1) = R(3,1);
            Ret(3,2) = R(3,2);
            Ret(3,3) = R(3,3);

            Ret(1, 4) = p(1);
            Ret(2, 4) = p(2);
            Ret(3, 4) = p(3);

            Ret(4, 1) = 0;
            Ret(4, 2) = 0;
            Ret(4, 3) = 0;
            Ret(4, 4) = 1;

            
            %prime tre righe della quarta colonna della direct kin
%           HOW TO COMPUTE THIS?????
%           % computing coordinates
            x = -t2*t3;
            y = t6;
            z = t5;

            pos = [x,y,z]';

            end
        
        
        function [Q] = inverse_kinematics(Q, err, mode)
            
            % Q : current config 1x4
            % err : error in pose
            % mode: 1 = visual servoing , 0 = go home proportional control
            
            % given error and current configuration returns next
            % configuration to converge to desired pose -> err=0
            
            J = kinematicsECM.compute_jacobian(Q);    
                       
            if mode==0                 
                v = 6.5*[1 1 1 1 0.2 0]*10^-2;
                alfa = zeros(4,3);
                J = pinv(J); % 4x3
                
            end
            
            if mode==1 
                % NOT USED 
                v = [0.3 0.3 0.3 0.3 0.3 0.3]*5; 
                alfa = diag(v);
            end

         
           
           %NON TORNANO LE DIMENSIONI QUINDI HO CAMBIATO LA DIM DI ERR!
           
           % computing newton method for inverse kinematics
           Q = Q' + J*(err);   %(1x4) = (4x1) + (4x3)(3x1);
           
        end
        
        function [Q] = getJoints(ID, vrep, h_joints)
            
            % getting current values of joints
            [~, q1] = vrep.simxGetJointPosition(ID, h_joints(1), vrep.simx_opmode_buffer);
            [~, q2] = vrep.simxGetJointPosition(ID, h_joints(2), vrep.simx_opmode_buffer);
            [~, q3] = vrep.simxGetJointPosition(ID, h_joints(3), vrep.simx_opmode_buffer);
            [~, q4] = vrep.simxGetJointPosition(ID, h_joints(4), vrep.simx_opmode_buffer);
            
            Q = [q1,q2,q3,q4];
            
        end
        
        function [] = setJoints(ID, vrep, h_joints, Q_new)
            
            % sending values in Q_new to Joints
            [~] = vrep.simxSetJointPosition(ID, h_joints(1), Q_new(1), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(ID, h_joints(2), Q_new(2), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(ID, h_joints(3), Q_new(3), vrep.simx_opmode_streaming);
            [~] = vrep.simxSetJointPosition(ID, h_joints(4), Q_new(4), vrep.simx_opmode_streaming);
            
            pause(0.01);
        end

        
        function [J] = compute_jacobian(Q)

            % computes Jacobian of current configuration
            % formula of Jacobian has been taken from distributed/dVKinematics.cpp
            % (Marco Ferro's file)
            
            q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);
          
            J = zeros(3,4);
            
            t2 = cos(q2);
            t3 = sin(q1);
            t4 = cos(q1);
            t5 = sin(q2);
            t6 = t2*t4;

           
%           jacobian(0,1) = q3*t2;
%           jacobian(0,2) = t5;
            J(1, 2) = t4;
            J(1, 4) = -t2*t3;
            J(2, 2) = t3;
            J(2, 4) = t6;
            J(3, 1) = 1.0;
            J(3, 4) = t5;
            
        end
    end
end