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

% end effector home pose
home_pose = [ 0.18474400 0.1270612  -0.0934647  -1.5118723   -0.65901488    0.38923407]'; % this is the one wrt rcm (used for inverse kin);

%	PROCESS LOOP

mode = 0;
spot = 0;
time = 0;

disp("------- STARTING -------");
while spot < 6 % spots are 5
    
    time = time +1;

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
        
        [~, ee_position]=vrep.simxGetObjectPosition(ID, h_j6, h_RCM, vrep.simx_opmode_streaming);
        [~, ee_orientation]=vrep.simxGetObjectOrientation(ID, h_j6, h_RCM, vrep.simx_opmode_streaming);
        
        ee_pose= [ee_position, ee_orientation]';
        
        % 2) COMPUTE ERROR
        err = utils.computeError(home_pose,ee_pose);
        
        % 3) EVALUATE EXIT CONDITION
        if norm(err(1:3),2)< 0.005
            
            at_home = true;
            spot = spot+1;
            mode = 1;
            fprintf(1, 'GOING TOWARD SPOT : %d \n', spot);
            pause(1);
            
        end
         
        % PLOT
        if( mod(time,5)==0)
            x = time/100;
            y = norm(err(1:3),2);
            % plot(x,y,'--b');
            stem(x,y,'-b');
            hold on
            grid on
            ylim( [0 0.25]);
            xlabel('time')
            ylabel('norm error')
            title('ERROR PLOT')
        end
                
        % 4) CORRECT AND UPDATE POSE
        
        % computing new configuration via inverse inverse kinematics
        Q = kinematicsRCM.inverse_kinematics(Q,err,0);
        
        % sending to joints
        [~] = vrep.simxSetJointPosition(ID, h_j1, Q(1), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j2, Q(2), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j3, Q(3), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j4, Q(4), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j5, Q(5), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j6, Q(6), vrep.simx_opmode_streaming);
        pause(0.01);
        
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
