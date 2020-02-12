%__________________________________________________________________________

%   INIT STUFF
%__________________________________________________________________________

cd(fileparts(mfilename('fullpath')));
clear;
close all;
clc;

pause(2);
%__________________________________________________________________________

% CONNECTION TO VREP
%__________________________________________________________________________

[ID,vrep] = utils.init_connection();

% COLLECTING HANDLES

% vision sensor
[~, h_VS] =vrep.simxGetObjectHandle(ID, 'Vision_sensor_ECM', vrep.simx_opmode_blocking);
[~, resolution, image] = vrep.simxGetVisionSensorImage(ID, h_VS, 0, vrep.simx_opmode_streaming);

% end effector
[~, h_EE] =vrep.simxGetObjectHandle(ID, 'EE', vrep.simx_opmode_blocking);

% end effector to follow
[~, h_followed] =vrep.simxGetObjectHandle(ID, 'FollowedDummy', vrep.simx_opmode_blocking);

% force sensor
[~, h_FS]=vrep.simxGetObjectHandle(ID, 'Force_sensor', vrep.simx_opmode_blocking);

% reference for direct kin
[~, h_RCM]=vrep.simxGetObjectHandle(ID, 'RCM_PSM1', vrep.simx_opmode_blocking);

% reference for direct kin
% first RRP joints
[~, h_j1] = vrep.simxGetObjectHandle(ID,'J1_PSM1',vrep.simx_opmode_blocking);
[~, h_j2] = vrep.simxGetObjectHandle(ID,'J2_PSM1',vrep.simx_opmode_blocking);
[~, h_j3] = vrep.simxGetObjectHandle(ID,'J3_PSM1',vrep.simx_opmode_blocking);

% second RRR joints
[~, h_j4] = vrep.simxGetObjectHandle(ID,'J1_TOOL1',vrep.simx_opmode_blocking);
[~, h_j5] = vrep.simxGetObjectHandle(ID,'J2_TOOL1',vrep.simx_opmode_blocking);
[~, h_j6] = vrep.simxGetObjectHandle(ID,'J3_TOOL1',vrep.simx_opmode_blocking);

% collection of all joint handles
h_joints = [h_j1; h_j2; h_j3; h_j4; h_j5; h_j6];

sync = utils.syncronize(ID, vrep, h_joints, h_RCM, h_VS, h_EE);

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

%__________________________________________________________________________

%   SETTINGS
%__________________________________________________________________________

% focal length (depth of the near clipping plane)
fl = 0.01;

% control gain in mode 0 (see below)
K = eye(6)*(10^-3);

% compliance matrix of manipulator
C = eye(6)*(10^-1);

% preallocating for speed
us_desired = zeros(4,5);
vs_desired = zeros(4,5);

sync=false; % used here below
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

% position and orientation of RCM wrt VS (remains costant)
% used in conversion of coordinates from VS to RCM
[~, vs2rcm_position]=vrep.simxGetObjectPosition(ID, h_RCM ,h_VS, vrep.simx_opmode_streaming);
[~, vs2rcm_orientation]=vrep.simxGetObjectOrientation(ID, h_RCM ,h_VS, vrep.simx_opmode_streaming);
vs2rcm = [vs2rcm_position';vs2rcm_orientation'];

%__________________________________________________________________________

%	PROCESS LOOP
%__________________________________________________________________________

mode = 1; % servoing control
spot = 1; % from which spot you start
time = 0; % time costant useful for plot ecc.

fprintf(2,'\n ******* STARTING ******* \n');

% % starting from zero config
% Q = zeros(6,1);
% kinematicsRCM.setJoints(ID, vrep, h_joints, Q);
% pause();

while spot<6
    
    time = time +1;
    
    Q = kinematicsRCM.getJoints(ID, vrep, h_joints);
    
    if mode==1
        
        %__________________________________________________________________________
        
        %	1) FEATURE EXTRACTION
        %__________________________________________________________________________
        
        us_ee = zeros(4,1);
        vs_ee = zeros(4,1);
        zs_ee = zeros(4,1);
        
        % GETTING CURRECT POSITION OF EE IN IMAGE PLANE
        for b=1:4 % balls
            while ~sync  % until i dont get valid values
                [~, l_position]=vrep.simxGetObjectPosition(ID, h_L_EE(b), h_VS, vrep.simx_opmode_streaming);
                sync = norm(l_position,2)~=0;
            end
            sync=false;
            
            zs_ee(b)= l_position(3);
            us_ee(b)= fl*l_position(1)/l_position(3);
            vs_ee(b)= fl*l_position(2)/l_position(3);
            
        end
                
        %__________________________________________________________________________
        
        %	2) COMPUTING INTERACTION MATRIX AND COMPUTING IMAGE ERROR
        %__________________________________________________________________________
                
        % building the jacobian
        L = [ utils.build_point_jacobian(us_ee(1),vs_ee(1),zs_ee(1),fl); ...
            utils.build_point_jacobian(us_ee(2),vs_ee(2),zs_ee(2),fl); ...
            utils.build_point_jacobian(us_ee(3),vs_ee(3),zs_ee(3),fl); ...
            utils.build_point_jacobian(us_ee(4),vs_ee(4),zs_ee(4),fl)];
        
        % computing the error
        err_image= [us_desired(1,spot)-us_ee(1); ...
            vs_desired(1,spot)-vs_ee(1); ...
            us_desired(2,spot)-us_ee(2); ...
            vs_desired(2,spot)-vs_ee(2); ...
            us_desired(3,spot)-us_ee(3); ...
            vs_desired(3,spot)-vs_ee(3); ...
            us_desired(4,spot)-us_ee(4); ...
            vs_desired(4,spot)-vs_ee(4)];
        
        % evaluating exit condition
        if norm(err_image,2)<=10^-4
            if spot == 5 % last spot
                break;
            end
            mode = 0;
            pause(2);
            disp("---------- OK ----------");
            continue;
        end
        
        %__________________________________________________________________________
        
        %	3) CORRECTING THE ERROR VIA FORCE FEEDBACK (null when no force)
        %__________________________________________________________________________
                
        while ~sync
            [~, ~, force, torque]=vrep.simxReadForceSensor(ID, h_FS, vrep.simx_opmode_streaming);
            sync = true; %norm(force,2)~=0;
        end
        sync=false;
        
        force_torque=[force'; torque'];
        force_torque=round(force_torque,2);
        
        force_correction = L*C*(force_torque_d-force_torque);
        err_image = err_image + force_correction;
        
        %__________________________________________________________________________
        
        %	4) COMPUTING DISPLACEMENT
        %__________________________________________________________________________
        
        % computing the displacement
        ee_displacement_VS = -K*pinv(L)*err_image;
        
        if norm(ee_displacement_VS,2)<10^-2.5 %10^-2.9
            ee_displacement_VS = (ee_displacement_VS/norm(ee_displacement_VS,2))*10^-2.5;
        end
        
        %__________________________________________________________________
        
        %	5) GETTING THE POSE WRT VISION SENSOR [VS]
        %__________________________________________________________________
        
        % getting the current pose wrt VS
        [~, ee_position_VS] = vrep.simxGetObjectPosition(ID, h_EE, h_VS, vrep.simx_opmode_streaming);
        [~, ee_orientation_VS] = vrep.simxGetObjectOrientation(ID, h_EE, h_VS, vrep.simx_opmode_streaming);
        
        ee_pose_VS = [ee_position_VS, ee_orientation_VS]';
        
        %__________________________________________________________________
        
        %	6) GETTING NEXT POSE WRT VS
        %__________________________________________________________________
        
        % updating the pose
        next_ee_pose_VS = ee_pose_VS + ee_displacement_VS;
        
        %__________________________________________________________________
        
        %	7) GETTING THE POSE WRT RCM frame [RCM]
        %__________________________________________________________________
        
        % this is the position of ee_pose wrt RCM frame
        % vs2rcm : relative position and orientation of RCM wrt VS
        ee_wrt_RCM = utils.getPoseInRCM(vs2rcm, ee_pose_VS);
        
        [~, ee_position_RCM]=vrep.simxGetObjectPosition(ID, h_EE, h_RCM, vrep.simx_opmode_streaming);
        [~, ee_orientation_RCM]=vrep.simxGetObjectOrientation(ID, h_EE, h_RCM, vrep.simx_opmode_streaming);
        
        % just used to check if transformation is good
        ee_pose_RCM = [ee_position_RCM, ee_orientation_RCM]';
        
        % ee_wrt_RCM (obtained using hom. transf.) and
        % real ee_pose_RCM are very close.
        % Check this statement here below.
        
        % check of prediction:
        %         tranformation_err = utils.computeError(ee_pose_RCM,ee_wrt_RCM);
        %
        %         % PLOT
        %         if( mod(time,10)==0)
        %             x = time/100;
        %             y = norm(tranformation_err(3:6),2);
        %             % plot(x,y,'--b');
        %             stem(x,y,'-b');
        %             % plot(prediction_err);
        %             hold on
        %             grid on
        %             ylim( [0 0.00001]);
        %             xlabel('time')
        %             ylabel('err')
        %             title('how bad is my prediction')
        %         end
        
        %__________________________________________________________________
        
        %	8) GETTING NEXT POSE WRT RCM
        %__________________________________________________________________
        
        % this is the next position of ee_pose wrt RCM frame
        next_ee_wrt_RCM = utils.getPoseInRCM(vs2rcm, next_ee_pose_VS);
        
        %__________________________________________________________________
        
        %	9) COMPUTING ERROR WRT RCM
        %__________________________________________________________________
        
        error_RCM = utils.computeError(next_ee_wrt_RCM, ee_wrt_RCM);
        
        %__________________________________________________________________
        
        %	10) SENDING NEW VALUES TO PSM via INVERSE KINEMATICS
        %__________________________________________________________________
        
        % computing the new configuration via inverse inverse kinematics
        Q_new = kinematicsRCM.inverse_kinematics(Q, error_RCM, mode);
        
        % sending to joints
        kinematicsRCM.setJoints(ID, vrep, h_joints, Q_new);
        
        %__________________________________________________________________
                
        % PLOT IMAGE PLANE
        if( mod(time,10)==0)
            
            subplot(2,1,1)
            % plotting current position
            plot = scatter( [us_ee(1), us_ee(2), us_ee(3), us_ee(4)],...
                [vs_ee(1), vs_ee(2), vs_ee(3), vs_ee(4)], 'o','k');
            
            % plotting desired position
            plot = scatter( [us_desired(1), us_desired(2), us_desired(3), us_desired(4) ],...
                [vs_desired(1), vs_desired(2), vs_desired(3), vs_desired(4)], 'r', 'o','filled');
            
            hold on
            grid on
            title('image error convergence')
            
            subplot(2,1,2)
            [~, resolution, image] = vrep.simxGetVisionSensorImage(ID, h_VS, 0, vrep.simx_opmode_buffer);
            imshow(image);

        end
        
    elseif mode==0
        
        % see testing_mode0
        break
    end
    pause(0.05);
end

fprintf(2,' \n **** PROCESS ENDED ***** \n');
