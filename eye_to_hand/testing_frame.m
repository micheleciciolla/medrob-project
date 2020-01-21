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

% end effector
[~, h_EE] =vrep.simxGetObjectHandle(ID, 'EE', vrep.simx_opmode_blocking);

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


%__________________________________________________________________________

%	PROCESS LOOP
%__________________________________________________________________________


fprintf(2,'\n ******* STARTING ******* \n');

% % starting from zero config
Q = zeros(6,1);
kinematicsRCM.setJoints(ID, vrep, h_joints, Q);
pause();