cd(fileparts(mfilename('fullpath')));
clear;
close all;
clc;

pause(3);
%%
% CONNECTION TO VREP
%%

[ID,vrep] = utils.init_connection();

% COLLECTING HANDLES

% vision sensor
[~, h_BP] =vrep.simxGetObjectHandle(ID, 'L4_visual_ECM', vrep.simx_opmode_blocking);

% as end effector Consider the lens of vs
[~, h_VS] =vrep.simxGetObjectHandle(ID, 'ECM_view', vrep.simx_opmode_blocking);

[~, h_PSM]=vrep.simxGetObjectHandle(ID, 'EE', vrep.simx_opmode_blocking);

% reference for direct kin
%joints (R R P R)
[~, h_j1] = vrep.simxGetObjectHandle(ID,'J1_ECM',vrep.simx_opmode_blocking);
[~, h_j2] = vrep.simxGetObjectHandle(ID,'J2_ECM',vrep.simx_opmode_blocking);
[~, h_j3] = vrep.simxGetObjectHandle(ID,'J3_ECM',vrep.simx_opmode_blocking);
[~, h_j4] = vrep.simxGetObjectHandle(ID,'J4_ECM',vrep.simx_opmode_blocking);


% collection of all joint handles
h_joints = [h_j1; h_j2; h_j3; h_j4];

sync = utils.syncronizeECM(ID, vrep, h_joints, h_BP, h_VS, h_PSM);

if sync
    fprintf(1,'Synchronize: OK... \n');
    pause(1);
end

kinematicsECM.setJoints(ID, vrep, h_joints, zeros(4,1));
pause(3);
Q = kinematicsECM.getJoints(ID, vrep, h_joints);

[increment, time, process_completed] = deal(0.03,0, false);

while time < 1000
    
    time = time +1;
    Q = kinematicsECM.getJoints(ID, vrep, h_joints);
    error = [-0.2000   -0.2000    0.2350   -0.2000] - Q;
    Q =  Q + increment*(error);
    
    kinematicsECM.setJoints(ID, vrep, h_joints, Q)
    
end

