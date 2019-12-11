
% script to test movement of EE using inverse and direct kinematics

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

[ID,vrep] = init_connection();


%%
% COLLECTING HANDLES
%%

% end effector attached dummy
[~, h_EE]=vrep.simxGetObjectHandle(ID, 'FollowedDummy', vrep.simx_opmode_blocking);

% first RRP joints
[~, h_j1] = vrep.simxGetObjectHandle(ID,'J1_PSM1',vrep.simx_opmode_blocking);
[~, h_j2] = vrep.simxGetObjectHandle(ID,'J1_PSM1',vrep.simx_opmode_blocking);
[~, h_j3] = vrep.simxGetObjectHandle(ID,'J1_PSM1',vrep.simx_opmode_blocking);

% second RRR joints
[~, h_j4] = vrep.simxGetObjectHandle(ID,'J1_TOOL1',vrep.simx_opmode_blocking);
[~, h_j5] = vrep.simxGetObjectHandle(ID,'J2_TOOL1',vrep.simx_opmode_blocking);
[~, h_j6] = vrep.simxGetObjectHandle(ID,'J3_TOOL1',vrep.simx_opmode_blocking);

% grippers
[~, h_7sx] = vrep.simxGetObjectHandle(ID,'J3_sx_TOOL1',vrep.simx_opmode_blocking);
[~, h_7dx] = vrep.simxGetObjectHandle(ID,'J3_dx_TOOL1',vrep.simx_opmode_blocking);

pause(0.1);

% end effector home pose
ee_pose_d=[ -1.5 ;   -4.07e-2;    +6.54e-1;  3.14;         0;         0];

% control gain
H = eye(6)*10^-1;

not_reached = true;

% collection of handlers
joints = [h_j1,h_j2,h_j3,h_j4,h_j5,h_j6]; % to-do USE indices in syncronize function
gripper = [h_7sx,h_7dx];

% syncronization phase
[sync] = syncronize( ID , vrep, h_EE, h_j1, h_j2, h_j3, h_j4, h_j5, h_j6,h_7sx, h_7dx);

if sync
    disp("Syncronized.");
    pause(1);
end

disp("------- STARTING -------");
%%
%	PROCESS LOOP
%%
while not_reached && sync
    
    % get current simulation time
    time = vrep.simxGetLastCmdTime(ID) / 1000.0;
    
    % getting the current pose
    [~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_buffer);
    [~, ee_orientation]=vrep.simxGetObjectOrientation(ID, h_EE, -1, vrep.simx_opmode_buffer);
    
    % getting current values of joints
    [~, pos_j1_psm]=vrep.simxGetJointPosition(ID,h_j1,vrep.simx_opmode_buffer);
    [~, pos_j2_psm]=vrep.simxGetJointPosition(ID,h_j2,vrep.simx_opmode_buffer);
    [~, pos_j3_psm]=vrep.simxGetJointPosition(ID,h_j3,vrep.simx_opmode_buffer);
    [~, pos_j1_tool]=vrep.simxGetJointPosition(ID,h_j4,vrep.simx_opmode_buffer);
    [~, pos_j2_tool]=vrep.simxGetJointPosition(ID,h_j5,vrep.simx_opmode_buffer);
    [~, pos_j3_tool]=vrep.simxGetJointPosition(ID,h_j6,vrep.simx_opmode_buffer);
    
    % getting current values of gripper
    [~, pos_gripper_sx]=vrep.simxGetJointPosition(ID,h_7sx,vrep.simx_opmode_buffer);
    [~, pos_gripper_dx]=vrep.simxGetJointPosition(ID,h_7dx,vrep.simx_opmode_buffer);
    
    %     if(mod(time,2)==0)
    %         disp(pos_j1_psm);
    %     end
    
    ee_pose= [ee_position, ee_orientation]';
    
    % computing the error
    err=[ee_pose_d(1:3) - ee_pose(1:3); angdiff(ee_pose(4:6), ee_pose_d(4:6)) ];
    
    
    % computing the displacement
    ee_displacement = H*err;
    
    %updating the pose
    ee_pose= ee_pose + ee_displacement;
    [~]= vrep.simxSetObjectPosition(ID, h_EE, -1, ee_pose(1:3), vrep.simx_opmode_streaming);
    [~]= vrep.simxSetObjectOrientation(ID, h_EE, -1, ee_pose(4:6), vrep.simx_opmode_streaming);
    
    % evaluating exit condition
    if norm(err)<=10^-3 || (time > 40)
        disp("Position reached");
        not_reached = false;
        compute_grasp(ID, h_7sx, h_7dx, pos_gripper_sx, pos_gripper_dx, vrep);
        compute_square(ID, vrep, h_EE);
    end
    
end

disp("############ PROCESS ENDED ############");

disp("Disconneting...");

pause(5);
vrep.simxStopSimulation(ID, vrep.simx_opmode_oneshot);

%%
%	FUNCTIONS
%%


function [clientID,vrep] = init_connection()

fprintf(1,'START...  \n');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
fprintf(1,'client %d\n', clientID);
if (clientID > -1)
    fprintf(1,'Connection: OK... \n');
else
    fprintf(2,'Connection: ERROR \n');
    return;
end
end

function [sync]  = syncronize(clientID , vrep, h_EE, h_j1_PSM, h_j2_PSM, h_j3_PSM, h_j1_TOOL, h_j2_TOOL, h_j3_TOOL, h_sx_GRIPPER, h_dx_GRIPPER)
sync = false;

while ~sync
    [~, ee_position]=vrep.simxGetObjectPosition(clientID, h_EE, -1, vrep.simx_opmode_streaming);
    sync = norm(ee_position,2)~=0;
end
sync=false;
while ~sync
    [~, ee_orientation]=vrep.simxGetObjectOrientation(clientID, h_EE, -1, vrep.simx_opmode_streaming);
    sync = norm(ee_orientation,2)~=0;
    
end
sync=false;


while ~sync
    % i dont need them all, just one to check non zero
    [~,~] = vrep.simxGetJointPosition(clientID, h_j1_PSM, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(clientID, h_j2_PSM, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(clientID, h_j3_PSM, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(clientID, h_j1_TOOL, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(clientID, h_j2_TOOL, vrep.simx_opmode_streaming);
    
    [~,~] = vrep.simxGetJointPosition(clientID, h_sx_GRIPPER, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(clientID, h_dx_GRIPPER, vrep.simx_opmode_streaming);
    
    [~,pos_j3_tool]=vrep.simxGetJointPosition(clientID,h_j3_TOOL,vrep.simx_opmode_streaming);
    
    sync = norm(pos_j3_tool,2)~=0;
    
end
end
function [] = compute_square(ID, vrep, h_EE)

r = 0.005;
[~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_buffer);
x = ee_position(1);
y = ee_position(2);
z = ee_position(3);

% forward
while ee_position(2) < (y + 0.2)
    
    pose = [x ,ee_position(2) + r, z];
    [~]= vrep.simxSetObjectPosition(ID, h_EE, -1, pose, vrep.simx_opmode_streaming);
    pause(0.05);
    [~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_buffer);
    
end

[~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_buffer);
x = ee_position(1);
y = ee_position(2);
z = ee_position(3);

% right
while ee_position(1) < (x + 0.2)
    
    pose = [ee_position(1) + r ,y, z];
    [~]= vrep.simxSetObjectPosition(ID, h_EE, -1, pose, vrep.simx_opmode_streaming);
    pause(0.05);
    [~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_buffer);
    
end

[~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_buffer);
x = ee_position(1);
y = ee_position(2);
z = ee_position(3);

% down
while ee_position(2) > (y - 0.2)
    
    pose = [x ,ee_position(2) - r, z];
    [~]= vrep.simxSetObjectPosition(ID, h_EE, -1, pose, vrep.simx_opmode_streaming);
    pause(0.05);
    [~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_buffer);
    
end

[~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_buffer);
x = ee_position(1);
y = ee_position(2);
z = ee_position(3);

% left
while ee_position(1) > (x - 0.2)
    
    pose = [ee_position(1) - r ,y, z];
    [~]= vrep.simxSetObjectPosition(ID, h_EE, -1, pose, vrep.simx_opmode_streaming);
    pause(0.05);
    [~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_buffer);
    
end


end
function [] = compute_grasp(clientID, h_7sx, h_7dx, pos_gripper_sx, pos_gripper_dx, vrep)
% to complete

sx = vrep.simxGetJointPosition(clientID,h_7sx,vrep.simx_opmode_streaming);
dx = vrep.simxGetJointPosition(clientID,h_7dx,vrep.simx_opmode_streaming);

% open
while sx < 3.14/4
    [~] = vrep.simxSetJointPosition(clientID, h_7sx, sx, vrep.simx_opmode_streaming);
    sx = sx + 0.02;
    [~] = vrep.simxSetJointPosition(clientID, h_7dx, sx, vrep.simx_opmode_streaming);
    dx = dx + 0.02;
    pause(0.05);
end

pause(1);

% close
while sx > 0
    [~] = vrep.simxSetJointPosition(clientID, h_7sx, sx, vrep.simx_opmode_streaming);
    sx = sx - 0.02;
    [~] = vrep.simxSetJointPosition(clientID, h_7dx, sx, vrep.simx_opmode_streaming);
    dx = dx - 0.02;
    pause(0.05);
end

end


function [J] = build_point_jacobian(u,v,z,fl)
J = [ -fl/z     0          u/z     (u*v)/fl        -(fl+(u^2)/fl)      v; ...
    0         -fl/z      v/z     (fl+(v^2)/fl)    -(u*v)/fl          -u];

end

%%
%	OLD
%%

%{
    %getting the features
    if ~isempty(image)
        fs=extract_features(image, grays);
    end
%}

function [fs] = extract_features(image, grays)
%
fs=zeros(4,1);
%
rimage=image(:,:,1);
gimage=image(:,:,2);
bimage=image(:,:,3);
%
for k=1:4
    %
    raw = (rimage==grays(k) & gimage==grays(k) & bimage==grays(k));
    %
    [J,I]=ind2sub(size(image),find(raw));
    %
    jmin=min(J);
    jmax=max(J);
    imin=min(I);
    imax=max(I);
    %
    fs(k,[1, 2])=[jmin+(jmax-jmin)/2, imin+(imax-imin)/2];
    %
end
end