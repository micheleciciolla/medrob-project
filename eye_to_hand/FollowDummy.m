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

% COLLECTING HANDLES

% vision sensor
[~, h_VS] =vrep.simxGetObjectHandle(ID, 'Vision_sensor_ECM', vrep.simx_opmode_blocking);

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

% USED TO WAIT TO RECEIVE NON-ZERO VALUES FROM followed dummy handle
% TO-DO: put it inside syncronize
sync=false;
while ~sync  
    [~, followed_pos]=vrep.simxGetObjectPosition(ID, h_followed, h_RCM, vrep.simx_opmode_streaming);
    [~, followed_orient]=vrep.simxGetObjectPosition(ID, h_followed, h_RCM, vrep.simx_opmode_streaming);
    sync = norm(followed_orient,2)~=0;
end

% end effector home pose wrt rcm
mode = 0;
spot = 0;
time = 0;
figure();

% starting from zero config
Q = zeros(6,1);
kinematicsRCM.setJoints(ID, vrep, h_joints, Q);
pause(1);


fprintf(2,'\n ******* STARTING ******* \n');


% home_pose = [ -1.5413e+0;   -4.0699e-2;    +7.2534e-1;  -1.80e+2;         0;         0];

% home_pose = [ followed_pos'; followed_orient'];
% fprintf(1,'\n ******* CLIK TO START ******* \n');
% 
% pause();

while spot < 6 % spots are 5
    
    [~, followed_pos]=vrep.simxGetObjectPosition(ID, h_followed, h_RCM, vrep.simx_opmode_streaming);
    [~, followed_orient]=vrep.simxGetObjectPosition(ID, h_followed, h_RCM, vrep.simx_opmode_streaming);
    goal_pose = [ followed_pos'; followed_orient'];

    time = time +1;
    
    Q = kinematicsRCM.getJoints(ID, vrep, h_joints);
    
    if mode == 0
        
        % 1) READ CURRENT POSE OF joint 6 or h_EE wrt RCM frame
        
        [~, ee_position]=vrep.simxGetObjectPosition(ID, h_EE, h_RCM, vrep.simx_opmode_streaming);
        [~, ee_orientation]=vrep.simxGetObjectOrientation(ID, h_EE, h_RCM, vrep.simx_opmode_streaming);
        
        ee_pose= [ee_position, ee_orientation]';
        
        % 2) COMPUTE ERROR
        err = utils.computeError(goal_pose,ee_pose);
                
%         % 3) EVALUATE EXIT CONDITION
%         if norm(err(1:3),2)< 10^-3
%             spot = spot+1;
%             mode = 1;
%             fprintf(1, 'GOING TOWARD SPOT : %d \n', spot);
%             pause(1);
%             
%         end
        
        % 4) CORRECT AND UPDATE POSE
        
        % computing new configuration via inverse inverse kinematics
        Q = kinematicsRCM.inverse_kinematics(Q,err,mode);
        
        % sending to joints
        [~] = vrep.simxSetJointPosition(ID, h_j1, Q(1), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j2, Q(2), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j3, Q(3), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j4, Q(4), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j5, Q(5), vrep.simx_opmode_streaming);
        [~] = vrep.simxSetJointPosition(ID, h_j6, Q(6), vrep.simx_opmode_streaming);
        
        pause(0.01);
        
        % PLOT
        if( mod(time,8)==0)
            
            x = time/100;
            y = norm(err(4:6),2);
            % plot(x,y,'--b');
            subplot(2,1,1)
            stem(x,y,'-k');
            ylim( [0 0.5]);
            xlabel('time')
            ylabel('norm error')
            title('Orientation error')
            hold on
            grid on
            
            subplot(2,1,2)
            y1 = norm(err(1:3),2);
            stem(x,y1,'-k');
            ylim( [0 0.25]);
            xlabel('time')
            ylabel('norm error')
            title('Position error')
            hold on
            grid on
            
        end
        
    elseif mode == 1
        break;
    end
end

fprintf(2,' \n **** PROCESS ENDED ***** \n');
disp("This is your final pose ( EE wrt RCM ) :");
disp(ee_pose);

disp("goal pose wrt RCM");
disp(goal_pose);

disp("difference");
diff100 = goal_pose - ee_pose;
diff100 = round( diff100, 5);
disp(diff100);
fprintf(2,' \n **** differences between frames ***** \n');

