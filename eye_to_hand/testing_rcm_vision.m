% testing mode 1 (NOT WORKING ATM)

%%
%   INIT STUFF
%%
cd(fileparts(mfilename('fullpath')));
clear;
close all;
clc;

pause(3);
%%
% CONNECTION TO VREP
%%

[ID,vrep] = utils.init_connection();

%%
% COLLECTING HANDLES
%%

% vision sensor
[~, h_VS]=vrep.simxGetObjectHandle(ID, 'Vision_sensor_ECM', vrep.simx_opmode_blocking);

% force sensor
[~, h_FS]=vrep.simxGetObjectHandle(ID, 'Force_sensor', vrep.simx_opmode_blocking);

% reference for direct kin
% first RRP joints
[~, h_j1] = vrep.simxGetObjectHandle(ID,'J1_PSM1',vrep.simx_opmode_blocking);
[~, h_j2] = vrep.simxGetObjectHandle(ID,'J2_PSM1',vrep.simx_opmode_blocking);
[~, h_j3] = vrep.simxGetObjectHandle(ID,'J3_PSM1',vrep.simx_opmode_blocking);

% second RRR joints
[~, h_j4] = vrep.simxGetObjectHandle(ID,'J1_TOOL1',vrep.simx_opmode_blocking);
[~, h_j5] = vrep.simxGetObjectHandle(ID,'J2_TOOL1',vrep.simx_opmode_blocking);
[~, h_j6] = vrep.simxGetObjectHandle(ID,'J3_TOOL1',vrep.simx_opmode_blocking);

% grippers (not used atm)
[~, h_7sx] = vrep.simxGetObjectHandle(ID,'J3_sx_TOOL1',vrep.simx_opmode_blocking);
[~, h_7dx] = vrep.simxGetObjectHandle(ID,'J3_dx_TOOL1',vrep.simx_opmode_blocking);

% collection of all joint handles
h_joints = [h_j1; h_j2; h_j3; h_j4; h_j5; h_j6];

% reference for direct kin
[~, h_RCM]=vrep.simxGetObjectHandle(ID, 'RCM_PSM1', vrep.simx_opmode_blocking);

relativeToObjectHandle = h_RCM; % relative to which frame you want to know position of ee

[sync] = utils.syncronize(ID, vrep, h_joints, h_RCM, h_VS);
if sync
    fprintf(1,'Sycronization: OK... \n');
    pause(1);
end

% preallocating for speed
h_L = zeros(4,5); % here i save handles of landmarks
h_L_EE = zeros(4,5); % here i save handles of balls attacched to EE

% landmarks attached to goal positions :
% we have 5 location to achieve
% each location has 4 landmarks

for b=1:4 % each spot has 4 balls
    for s=1:5 % 5 total spots
        [~, h_L(b,s)]=vrep.simxGetObjectHandle(ID, ['Landmark', num2str(s), num2str(b)], vrep.simx_opmode_blocking);
    end
end

% landmarks attached to EE -> 'LandmarkEE1,2,3,4'
for b=1:4
    [~, h_L_EE(b)]=vrep.simxGetObjectHandle(ID, ['LandmarkEE', num2str(b)], vrep.simx_opmode_blocking);
end

%%
%   SETTINGS
%%

% focal length (depth of the near clipping plane)
fl = 0.01;

% control gain in mode 0 (see below)
K = eye(6)*(10^-2);

% control gain in mode 1 (see below)
H = eye(6)*(10^-1);

% compliance matrix of manipulator
C = eye(6)*(10^-1);

% preallocating for speed
us_desired = zeros(4,5);
vs_desired = zeros(4,5);
sync=false;

% desired features EXTRACTION
for b=1:4 % balls
    for s=1:5 % spots
        while ~sync % until i dont get valid values
            [~, l_position]=vrep.simxGetObjectPosition(ID, h_L(b,s), h_VS, vrep.simx_opmode_streaming);
            sync = norm(l_position,2)~=0;
        end
        sync=false;
        
        % here you have all landmark positions in image plane
        us_desired(b,s)= fl*l_position(1)/l_position(3);
        vs_desired(b,s)= fl*l_position(2)/l_position(3);
        
    end
end

% null desired force and torque
force_torque_d=zeros(6,1);

% PROCESS LOOP
mode = 1;
time = 0;
% start from landmark at spot+1
spot = 1;

% loop
disp("------- STARTING -------");

% distance between origins: VS_frame and RCM_frame (from VS -> RCM)
% used below to get pose wrt RCM

% should be costant
[~, vs2rcm_position]=vrep.simxGetObjectPosition(ID, h_RCM ,h_VS, vrep.simx_opmode_streaming);
[~, vs2rcm_orientation]=vrep.simxGetObjectOrientation(ID, h_RCM ,h_VS, vrep.simx_opmode_streaming);
vs2rcm = [vs2rcm_position';vs2rcm_orientation'];

while spot<6
    
    time = time +1;
    
    % getting current values of joints
    [~, q1]=vrep.simxGetJointPosition(ID,h_j1,vrep.simx_opmode_buffer);
    [~, q2]=vrep.simxGetJointPosition(ID,h_j2,vrep.simx_opmode_buffer);
    [~, q3]=vrep.simxGetJointPosition(ID,h_j3,vrep.simx_opmode_buffer);
    [~, q4]=vrep.simxGetJointPosition(ID,h_j4,vrep.simx_opmode_buffer);
    [~, q5]=vrep.simxGetJointPosition(ID,h_j5,vrep.simx_opmode_buffer);
    [~, q6]=vrep.simxGetJointPosition(ID,h_j6,vrep.simx_opmode_buffer);
    Q = [q1,q2,q3,q4,q5,q6];
    
    if mode==1
        
        % _________________________________________________________________
        % _________________________________________________________________
        
        % 1) FEATURE EXTRACTION OF EE
        
        us_current = zeros(4,1);
        vs_currect = zeros(4,1);
        zs_current = zeros(4,1);
        
        % GETTING CURRECT POSITION OF EE IN IMAGE PLANE
        for b=1:4 % balls
            while ~sync  % until i dont get valid values
                [~, l_position]=vrep.simxGetObjectPosition(ID, h_L_EE(b), h_VS, vrep.simx_opmode_streaming);
                sync = norm(l_position,2)~=0;
            end
            sync=false;
            
            zs_current(b)= l_position(3);
            us_current(b)= fl*l_position(1)/l_position(3);
            vs_currect(b)= fl*l_position(2)/l_position(3);
            
        end
        
        % _________________________________________________________________
        % _________________________________________________________________
        
        % 2) BUILDING POINT JACOBIAN
               
        % building the jacobian
        L = [ utils.build_point_jacobian(us_current(1),vs_currect(1),zs_current(1),fl); ...
            utils.build_point_jacobian(us_current(2),vs_currect(2),zs_current(2),fl); ...
            utils.build_point_jacobian(us_current(3),vs_currect(3),zs_current(3),fl); ...
            utils.build_point_jacobian(us_current(4),vs_currect(4),zs_current(4),fl)];

        % _________________________________________________________________
        % _________________________________________________________________
        
        % 3) COMPUTING IMAGE ERROR
        
        % computing the error
        err= [  us_desired(1,spot)-us_current(1); ...
                vs_desired(1,spot)-vs_currect(1); ...
                us_desired(2,spot)-us_current(2); ...
                vs_desired(2,spot)-vs_currect(2); ...
                us_desired(3,spot)-us_current(3); ...
                vs_desired(3,spot)-vs_currect(3); ...
                us_desired(4,spot)-us_current(4); ...
                vs_desired(4,spot)-vs_currect(4)        
              ];

        % _________________________________________________________________
        % _________________________________________________________________
        
        % 4) EVALUATING EXIT CONDITIONS
        
        % evaluating exit condition
        if norm(err,2)<=10^-4
            if spot==5 % last spot
                break;
            end
            mode=0;
            pause(2);
            fprintf(1, 'REACHED SPOT : %d \n', spot);
            continue;
        end
                       
        % _________________________________________________________________
        
        % 6) COMPUTING THE DISPLACEMENT    
        % _________________________________________________________________

        
        % computing the displacement
        ee_displacement_VS = K*pinv(-L)*err;
        
        if norm(ee_displacement_VS,2)<10^-2.5 %10^-2.9
            ee_displacement_VS = (ee_displacement_VS/norm(ee_displacement_VS,2))*10^-2.5;
        end
         
        % _________________________________________________________________
        
        % 7) GETTING THE POSE WRT VISION SENSOR [VS]
                % _________________________________________________________________

                
        [~, ee_position_VS]=vrep.simxGetObjectPosition(ID, h_j6, h_VS, vrep.simx_opmode_streaming);
        [~, ee_orientation_VS]=vrep.simxGetObjectOrientation(ID, h_j6, h_VS, vrep.simx_opmode_streaming);
           
        ee_pose_VS = [ee_position_VS, ee_orientation_VS]';
        
        % updating the pose
        next_ee_pose_VS = ee_pose_VS + ee_displacement_VS;
        
        if norm(ee_displacement_VS,2)<10^-2.5
            % non so cosa faccia ma l' ho preso dallo script originale
            ee_displacement_VS = (ee_displacement_VS/norm(ee_displacement_VS,2))*10^-2.5;
        end
                
        % _________________________________________________________________        
        
        % 8) GETTING THE POSE WRT RCM
        
                % _________________________________________________________________

        % this is the position of ee_pose wrt RCM frame
        ee_wrt_RCM = utils.getPoseInRCM(vs2rcm,ee_pose_VS);
        
        
        [~, ee_position]=vrep.simxGetObjectPosition(ID, h_j6, h_RCM, vrep.simx_opmode_streaming);
        [~, ee_orientation]=vrep.simxGetObjectOrientation(ID, h_j6, h_RCM, vrep.simx_opmode_streaming);
        ee_pose= [ee_position, ee_orientation]';
        
        % this is the next position of ee_pose wrt RCM frame
        next_ee_wrt_RCM = utils.getPoseInRCM(vs2rcm,next_ee_pose_VS);      
       
        % error_rcm = zeros(6,1);
        % error_rcm(3) = -0.01; % simulo errore solo su z 
        
        
        
        error_rcm = utils.computeError(next_ee_wrt_RCM, ee_pose);
        plot(error_rcm,'*');
        title("error_r_c");
        hold on
        grid on
                % _________________________________________________________________

        % 9) CORRECT AND UPDATE POSE via INVERSE KINEMATICS
                % _________________________________________________________________

        % computing the new configuration via inverse inverse kinematics
        Q_new = kinematicsRCM.inverse_kinematics(Q, error_rcm, mode);
        
        % sending to joints
        [~] = vrep.simxSetJointPosition(ID, h_j1, Q_new(1), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j2, Q_new(2), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j3, Q_new(3), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j4, Q_new(4), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j5, Q_new(5), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j6, Q_new(6), vrep.simx_opmode_streaming);
        
        pause(0.01);
        
        
    elseif mode == 0
        
        % see testing_mode0.m
        break
        
    end
end

disp("############ PROCESS ENDED ############");
