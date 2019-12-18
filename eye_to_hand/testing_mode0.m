% testing mode 0 with inverse kin



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


% For sync purposes (see below)
sync = false; 

%%
% COLLECTING HANDLES
%%

% vision sensor
[~, h_VS]=vrep.simxGetObjectHandle(ID, 'Vision_sensor_ECM', vrep.simx_opmode_blocking);

% force sensor
[~, h_FS]=vrep.simxGetObjectHandle(ID, 'Force_sensor', vrep.simx_opmode_blocking);

% end effector
[~, h_EE]=vrep.simxGetObjectHandle(ID, 'FollowedDummy', vrep.simx_opmode_blocking);


% here you have to insert handles of RCM and h_j6 (maybe) to get desired
% pose in relative frame (not absolute -1)



% reference for direct kin
% first RRP joints
[~, h_j1] = vrep.simxGetObjectHandle(ID,'J1_PSM1',vrep.simx_opmode_blocking);
[~, h_j2] = vrep.simxGetObjectHandle(ID,'J2_PSM1',vrep.simx_opmode_blocking);
[~, h_j3] = vrep.simxGetObjectHandle(ID,'J3_PSM1',vrep.simx_opmode_blocking);

% second RRR joints
[~, h_j4] = vrep.simxGetObjectHandle(ID,'J1_TOOL1',vrep.simx_opmode_blocking);
[~, h_j5] = vrep.simxGetObjectHandle(ID,'J2_TOOL1',vrep.simx_opmode_blocking);
[~, h_j6] = vrep.simxGetObjectHandle(ID,'J3_TOOL1',vrep.simx_opmode_blocking);

% grippers
[~, h_7sx] = vrep.simxGetObjectHandle(ID,'J3_sx_TOOL1',vrep.simx_opmode_blocking);
[~, h_7dx] = vrep.simxGetObjectHandle(ID,'J3_dx_TOOL1',vrep.simx_opmode_blocking);

% reference for direct kin
[~, h_RCM]=vrep.simxGetObjectHandle(ID, 'RCM_PSM1', vrep.simx_opmode_blocking);

relativeToObjectHandle = h_RCM; % relative to which frame you want to know position of ee

[sync] = syncronize( ID , vrep, h_j1, h_j2, h_j3, h_j4, h_j5, h_j6, h_7sx, h_7dx, h_RCM);
if sync
    disp("Syncronized.");
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

% landmarks colors (old)
% grays=[0.8; 0.6; 0.4; 0.2]*255;     %landmarks' gray shades

% focal length (depth of the near clipping plane)
fl = 0.01;

% control gain in mode 0 (see below)
K = eye(6)*(10^-2);

% control gain in mode 1 (see below)
H = eye(6)*(10^-1);

% compliance matrix of manipulator
COMPLIANCE = eye(6)*(10^-1);

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

% end effector home pose
home_pose = [ 0.09  0.035  -0.0938  -1.5702   -0.3370    0.7288]'; % this is the one wrt rcm (used for inverse kin);

Q = zeros(1,6);
%%
%	PROCESS LOOP
%%

% two possible control modes:
    % mode 0: go-to-home control mode; 
    % mode 1: visual servoing eye-on-hand control mode;
mode=0;

% mode 0 overview:
	% i) features and depth extraction

	% ii) building image jacobian and computing the error (vision-based only)
			
	% iii) adjusting the error (via the force-based infos)
	
    % iv) computing the ee displacement	
    
	% v) updating the pose

% mode 1 is just a Cartesian proportionale regulator

%start from landmark at spot+1
spot = 0;

% loop
disp("------- STARTING -------");

while spot < 6 % spots are 5
    
    time = vrep.simxGetLastCmdTime(ID) / 1000.0;

    % getting current values of joints
    [~, q1]=vrep.simxGetJointPosition(ID,h_j1,vrep.simx_opmode_buffer);
    [~, q2]=vrep.simxGetJointPosition(ID,h_j2,vrep.simx_opmode_buffer);
    [~, q3]=vrep.simxGetJointPosition(ID,h_j3,vrep.simx_opmode_buffer);
    [~, q4]=vrep.simxGetJointPosition(ID,h_j4,vrep.simx_opmode_buffer);
    [~, q5]=vrep.simxGetJointPosition(ID,h_j5,vrep.simx_opmode_buffer);
    [~, q6]=vrep.simxGetJointPosition(ID,h_j6,vrep.simx_opmode_buffer);    
    Q = [q1,q2,q3,q4,q5,q6];
    
    if mode == 1
        
        % TO BE COMPLETED
        
        break;
                
        
    elseif mode == 0
        
        % 1) READ CURRENT POSE OF joint 6 wrt RCM frame
        
        relativeToObjectHandle = h_RCM;
        
        [~, ee_position]=vrep.simxGetObjectPosition(ID, h_j6, relativeToObjectHandle, vrep.simx_opmode_streaming);
        [~, ee_orientation]=vrep.simxGetObjectOrientation(ID, h_j6, relativeToObjectHandle, vrep.simx_opmode_streaming);
        
        ee_pose= [ee_position, ee_orientation]';
        
        % 2) COMPUTE ERROR
        
        err=[home_pose(1:3) - ee_pose(1:3); angdiff(ee_pose(4:6), home_pose(4:6))];

        % 3) EVALUATE EXIT CONDITION
        
        if max(err)<=0.01
            at_home = true;
            spot = spot+1;
            mode = 1;
            fprintf(1, 'GOING TOWARD SPOT : %d \n', spot);
            pause(1);
        end
        
        x = time;
        y = norm(err,2);
        % plot(x,y,'--b');
        stem(x,y,'-b');
        hold on
        grid on
        ylim( [0 0.5]);
        xlabel('time')
        ylabel('norm error')
        title('ERROR PLOT')
        
        % 4) CORRECT AND UPDATE POSE
        
        % computing new configuration via inverse inverse kinematics
        Q = kinematicsRCM.inverse_kinematics(Q,err);
        
        % sending to joints
        [~] = vrep.simxSetJointPosition(ID, h_j1, Q(1), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j2, Q(2), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j3, Q(3), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j4, Q(4), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j5, Q(5), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j6, Q(6), vrep.simx_opmode_streaming);
        pause(0.2);
        
    end
end

disp("############ PROCESS ENDED ############");

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

function [sync]  = syncronize(ID , vrep, h_j1, h_j2, h_j3, h_j4, h_j5, h_j6, h_7sx, h_7dx, h_RCM)
% to be prettyfied -> you will receive in input just (clientID , vrep, handles)
% h_EE = handles(1)
% h_j1_PSM = handles(2)
% ...

% used to wait to receive non zero values from vrep model
% usually matlab and vrep need few seconds to send valid values

sync = false;

while ~sync
    % i dont need them all, just one to check non-zero returning values 
    [~,~] = vrep.simxGetJointPosition(ID, h_j1, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(ID, h_j2, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(ID, h_j3, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(ID, h_j4, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(ID, h_j5, vrep.simx_opmode_streaming);
    [~,some]=vrep.simxGetJointPosition(ID,h_j6,vrep.simx_opmode_streaming);
       
    [~,~] = vrep.simxGetJointPosition(ID, h_7sx, vrep.simx_opmode_streaming);
    [~,~] = vrep.simxGetJointPosition(ID, h_7dx, vrep.simx_opmode_streaming);
     
    [~,~] = vrep.simxGetJointPosition(ID, h_RCM, vrep.simx_opmode_streaming);
    
    [~, ee_position_relative]=vrep.simxGetObjectPosition(ID, h_j6, h_RCM, vrep.simx_opmode_streaming);
    [~, ee_orientation_relative]=vrep.simxGetObjectOrientation(ID, h_j6, h_RCM, vrep.simx_opmode_streaming);

    sync = norm(some,2)~=0;
end

end
