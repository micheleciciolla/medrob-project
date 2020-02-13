%--------------------------------------------------------------------------
%   INIT STUFF

cd(fileparts(mfilename('fullpath')));
clear;
close all;
clc;

pause(2);
%--------------------------------------------------------------------------
% CONNECTION TO VREP

[ID,vrep] = utils.init_connection();

% COLLECTING HANDLES

%--------------------------------------------------------------------------

% vision sensor
[~, h_VS] =vrep.simxGetObjectHandle(ID, 'ECM_view', vrep.simx_opmode_blocking);
%--------------------------------------------------------------------------

% end effector
[~, h_EE]=vrep.simxGetObjectHandle(ID, 'EE', vrep.simx_opmode_blocking);

% dummy followed
[~, h_Followed]=vrep.simxGetObjectHandle(ID, 'FollowedDummy', vrep.simx_opmode_blocking);
%--------------------------------------------------------------------------

% force sensor
[~, h_FS]=vrep.simxGetObjectHandle(ID, 'Force_sensor', vrep.simx_opmode_blocking);
%--------------------------------------------------------------------------

% rcm handle
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

%--------------------------------------------------------------------------

% collection of all joint handles
h_Joints = [h_j1; h_j2; h_j3; h_j4; h_j5; h_j6];

sync = utils.syncronize(ID, vrep, h_Joints, h_RCM, h_VS, h_Followed, h_EE);

if sync
    fprintf(1,'Sycronization: OK... \n');
    pause(1);
end

%--------------------------------------------------------------------------

% landmarks attached to goal positions :
% we have 5 location to achieve
% each location has 4 landmarks
% preallocating for speed

h_L = zeros(4,5); % here i save handles of landmarks
h_L_EE = zeros(4,5); % here i save handles of balls attacched to EE

% landmarks attached to spots
for b=1:4 % each spot has 4 balls
    for s=1:5 % 5 total spots
        [~, h_L(b,s)]=vrep.simxGetObjectHandle(ID, ['Landmark', num2str(s), num2str(b)], vrep.simx_opmode_blocking);
    end
end

% landmarks attached to DUMMY FOLLOWED -> 'LandmarkEE1,2,3,4'
for b=1:4
    [~, h_L_EE(b)]=vrep.simxGetObjectHandle(ID, ['LandmarkEE', num2str(b)], vrep.simx_opmode_blocking);
end

%%
%   SETTINGS
%%

% focal length (depth of the near clipping plane)
fl = 0.01;

% control gain in mode 1 (see below)
K = eye(6)*(10^-2)*0.2;

% control gain in mode 0 (see below)
H = eye(6)*(10^-1)*2;

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
        
        % here you have all landmark positions in image plane (perspective)
        us_desired(b,s)= fl*l_position(1)/l_position(3);
        vs_desired(b,s)= fl*l_position(2)/l_position(3);
        
    end
end

% null desired force and torque
force_torque_d=zeros(6,1);

home_pose_wrt_world = [ -1.54;   -4.069e-2;    +7.25e-1;  -1.79e+2; -4.2305e-02;         6.80022e-01];
% sending new pose of dummy attached to EE wrt RCM
[~]= vrep.simxSetObjectPosition(ID, h_Followed, -1, home_pose_wrt_world(1:3), vrep.simx_opmode_oneshot);
[~]= vrep.simxSetObjectOrientation(ID, h_Followed, -1, home_pose_wrt_world(4:6), vrep.simx_opmode_oneshot);

% starting from zero config
Q = zeros(6,1);
kinematicsRCM.setJoints(ID, vrep, h_Joints, Q);

pause(3);

%%
%	PROCESS LOOP
%%

% position and orientation of RCM wrt VS (should be costant)
% used in conversion of coordinates from VS to RCM

[~, vs2rcm_position]=vrep.simxGetObjectPosition(ID, h_RCM ,h_VS, vrep.simx_opmode_streaming);
[~, vs2rcm_orientation]=vrep.simxGetObjectOrientation(ID, h_RCM ,h_VS, vrep.simx_opmode_streaming);
vs2rcm = [vs2rcm_position';vs2rcm_orientation'];

%start from landmark at spot+1
spot = round(rand*5)
% first go to home pose in mode 0
mode = 1;
% time variable useful for plot ecc.
time = 0;
% if ghost reached position
ghost_reached = false;

fprintf(2,'******* STARTING ******* \n');

while spot<6
    
    if mode==1
        
        %	I) FEATURES and DEPTH EXTRACTION
        
        us_ee = zeros(4,1);
        vs_ee = zeros(4,1);
        zs_ee = zeros(4,1);
        
        % GETTING CURRECT POSITION OF EE IN IMAGE PLANE
        for b=1:4 % ee_balls
            while ~sync  % until i dont get valid values
                [~, l_position]=vrep.simxGetObjectPosition(ID, h_L_EE(b), h_VS, vrep.simx_opmode_streaming);
                sync = norm(l_position,2)~=0;
            end
            sync=false;
            
            zs_ee(b)= l_position(3);
            us_ee(b)= fl*l_position(1)/l_position(3);
            vs_ee(b)= fl*l_position(2)/l_position(3);
            
        end
        
        time = time +1;
        
        %%
        %	II) BUILDING the IMAGE JACOBIAN and COMPUTING THE ERROR (vision-based only)
        %%
        
        % building the jacobian
        L = [ utils.build_point_jacobian(us_ee(1),vs_ee(1),zs_ee(1),fl); ...
            utils.build_point_jacobian(us_ee(2),vs_ee(2),zs_ee(2),fl); ...
            utils.build_point_jacobian(us_ee(3),vs_ee(3),zs_ee(3),fl); ...
            utils.build_point_jacobian(us_ee(4),vs_ee(4),zs_ee(4),fl)];
        
        % computing the error
        err= [us_desired(1,spot)-us_ee(1); ...
            vs_desired(1,spot)-vs_ee(1); ...
            us_desired(2,spot)-us_ee(2); ...
            vs_desired(2,spot)-vs_ee(2); ...
            us_desired(3,spot)-us_ee(3); ...
            vs_desired(3,spot)-vs_ee(3); ...
            us_desired(4,spot)-us_ee(4); ...
            vs_desired(4,spot)-vs_ee(4)];
        
        if norm(err,2)<10^-4 && ghost_reached==false
            ghost_reached = true;
            disp("ghost reached");
        end
        
        %%
        %	III) ADJUSTING the ERROR (via the force-based infos)
        %
        
        while ~sync
            [~, ~, force, torque]=vrep.simxReadForceSensor(ID, h_FS, vrep.simx_opmode_streaming);
            sync = true; %norm(force,2)~=0;
        end
        sync=false;
        %
        force_torque=[force'; torque'];
        force_torque=round(force_torque,2);
        %
        force_correction = L*C*(force_torque_d-force_torque);
        err = err + force_correction;
        
        time = time +1;
        
        %__________________________________________________________________
        
        %	IV) COMPUTING the EE DISPLACEMENT
        %__________________________________________________________________
        
        % computing the displacement
        ee_displacement = K*pinv(-L)*err;
        
        if norm(ee_displacement,2)<10^-2.5 %10^-2.9
            ee_displacement = (ee_displacement/norm(ee_displacement,2))*10^-2.5;
        end
        
        %__________________________________________________________________
        %__________________________________________________________________
        
        %	V) UPDATING THE POSE
        
        % getting the current pose wrt VS
        while ~sync
            [~, ee_position_VS]=vrep.simxGetObjectPosition(ID, h_Followed, h_VS, vrep.simx_opmode_streaming);
            sync = norm(ee_position_VS,2)~=0;
        end
        sync=false;
        
        while ~sync
            [~, ee_orientation_VS]=vrep.simxGetObjectOrientation(ID, h_Followed, h_VS, vrep.simx_opmode_streaming);
            sync = norm(ee_orientation_VS,2)~=0;
        end
        sync=false;
        
        ee_pose_VS= [ee_position_VS, ee_orientation_VS]';
        
        % getting the new pose
        next_ee_pose_VS = ee_pose_VS + ee_displacement;
        
        % sending new pose of dummy attached to EE wrt VS
        
        % [~]= vrep.simxSetObjectPosition(ID, h_EE, h_VS, next_ee_pose_VS(1:3), vrep.simx_opmode_oneshot);
        % [~]= vrep.simxSetObjectOrientation(ID, h_EE, h_VS, next_ee_pose_VS(4:6), vrep.simx_opmode_oneshot);
        
        % getting next pose wrt RCM
        next_ee_wrt_RCM = utils.getPoseInRCM(vs2rcm, next_ee_pose_VS);
        
        % sending new pose of dummy attached to EE wrt RCM
        [~]= vrep.simxSetObjectPosition(ID, h_Followed, h_RCM, next_ee_wrt_RCM(1:3), vrep.simx_opmode_oneshot);
        [~]= vrep.simxSetObjectOrientation(ID, h_Followed, h_RCM, next_ee_wrt_RCM(4:6), vrep.simx_opmode_oneshot);
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                                                               %%
        %        HERE YOU FOLLOW DUMMY USING INVERSE KINEMATICS         %%
        %                                                               %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Q = kinematicsRCM.getJoints(ID, vrep, h_Joints);
        
        %reading where's my EE
        [~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, h_RCM, vrep.simx_opmode_streaming);
        [~, ee_orientation]=vrep.simxGetObjectOrientation(ID, h_EE, h_RCM, vrep.simx_opmode_streaming);
        ee_wrt_RCM= [ee_position, ee_orientation]';
        
        [~, dummy_position]=vrep.simxGetObjectPosition(ID, h_Followed, h_RCM, vrep.simx_opmode_streaming);
        [~, dummy_orientation]=vrep.simxGetObjectOrientation(ID, h_Followed, h_RCM, vrep.simx_opmode_streaming);
        
        dummy= [dummy_position, dummy_orientation]';
        
        % COMPUTE ERROR
        err = utils.computeError(dummy,ee_wrt_RCM);
        
        % computing new configuration via inverse inverse kinematics
        Q = kinematicsRCM.inverse_kinematics(Q,err,0);
        
        % sending to joints
        [~] = vrep.simxSetJointPosition(ID, h_j1, Q(1), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j2, Q(2), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j3, Q(3), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j4, Q(4), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j5, Q(5), vrep.simx_opmode_streaming);
        % sixth joint causes glityches. i'm disabling it
        % [~] = vrep.simxSetJointPosition(ID, h_j6, Q(6), vrep.simx_opmode_streaming);
        
        %evaluating exit condition
        if norm(err(1:3),2)<=10^-3 && ghost_reached
            if spot==4 % last spot
                break;
            end
            mode=0;
            
            disp("------------ OK ------------");
            
            close
            ghost_reached = false;
        end
        
        
    elseif mode==0
        
        %% TO BE COMPLETED %% 
        
        
    end               
    
end

fprintf(2,'**** PROCESS ENDED ***** \n');