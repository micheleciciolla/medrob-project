%%
%   INIT STUFF
%%
cd(fileparts(mfilename('fullpath')));
clear;
close all;
clc;

pause(2);
%%
% CONNECTION TO VREP
%%

[ID,vrep] = utils.init_connection();

%%
% COLLECTING HANDLES
%%

% end effector
[~, h_EE]=vrep.simxGetObjectHandle(ID, 'FollowedDummy', vrep.simx_opmode_blocking);

% vision sensor
[~, h_VS] =vrep.simxGetObjectHandle(ID, 'Vision_sensor_ECM', vrep.simx_opmode_blocking);

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

relativeToObjectHandle = h_RCM;

% collection of all joint handles
h_joints = [h_j1; h_j2; h_j3; h_j4; h_j5; h_j6];

[sync] = utils.syncronize(ID, vrep, h_joints, h_RCM, h_VS, h_EE);
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

% landmarks colors (old)
% grays=[0.8; 0.6; 0.4; 0.2]*255;     %landmarks' gray shades

% focal length (depth of the near clipping plane)
fl = 0.01;

% control gain in mode 1 (see below)
K = eye(6)*(10^-2)*1.5;

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
        
        % here you have all landmark positions in image plane
        us_desired(b,s)= fl*l_position(1)/l_position(3);
        vs_desired(b,s)= fl*l_position(2)/l_position(3);
        
    end
end

% null desired force and torque
force_torque_d=zeros(6,1);

% end effector home pose (THIS NEEDS TO BE CORRECTED)
ee_pose_d = [ -1.5413e+0;   -4.0699e-2;    +7.2534e-1;  -1.80e+2;         0;         0];
home_pose = [ 0.09 0.035 -0.0938 -1.458 -0.586 0.7929]; % this is the one wrt rcm (used for inverse kin);

%%
%	PROCESS LOOP
%%

% two possible control modes:
    % mode 0: go-to-home control mode; 
    % mode 1: visual servoing eye-on-hand control mode;
mode = 0;

% mode 0 overview:
	% i) features and depth extraction

	% ii) building image jacobian and computing the error (vision-based only)
			
	% iii) adjusting the error (via the force-based infos)
	
    % iv) computing the ee displacement	
    
	% v) updating the pose

% mode 1 is just a Cartesian proportionale regulator

% position and orientation of RCM wrt VS (should be costant)
% used in conversion of coordinates from VS to RCM
[~, vs2rcm_position]=vrep.simxGetObjectPosition(ID, h_RCM ,h_VS, vrep.simx_opmode_streaming);
[~, vs2rcm_orientation]=vrep.simxGetObjectOrientation(ID, h_RCM ,h_VS, vrep.simx_opmode_streaming);
vs2rcm = [vs2rcm_position';vs2rcm_orientation'];

%start from landmark at spot+1
spot = 0;

% loop
% disp("******* STARTING ******* \n");
fprintf(2,'******* STARTING ******* \n');

time = 0; % time variable, useful for plot ecc.
figure();
while spot<6
    
    if mode==1
                
        %%
        %	I) FEATURES and DEPTH EXTRACTION
        %%

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
        
        % norm(err,2)
        
        % evaluating exit condition
        if norm(err,2)<=10^-4
            if spot==4 % last spot
                break;
            end
            mode=0;       
            
            disp("---------- OK ----------");
            fprintf(2,'---------- PAUSED ---------- \n');

            pause();
            close
            continue;
        end
        %
        
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
        
        %% 
        %	IV) COMPUTING the EE DISPLACEMENT
              
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
            [~, ee_position_VS]=vrep.simxGetObjectPosition(ID, h_EE, h_VS, vrep.simx_opmode_streaming);
            sync = norm(ee_position_VS,2)~=0;
        end
        sync=false;
        
        while ~sync
            [~, ee_orientation_VS]=vrep.simxGetObjectOrientation(ID, h_EE, h_VS, vrep.simx_opmode_streaming);
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
        [~]= vrep.simxSetObjectPosition(ID, h_EE, h_RCM, next_ee_wrt_RCM(1:3), vrep.simx_opmode_oneshot);
        [~]= vrep.simxSetObjectOrientation(ID, h_EE, h_RCM, next_ee_wrt_RCM(4:6), vrep.simx_opmode_oneshot);
        
        % PLOT
        if( mod(time,40)==0)
            
            subplot(2,1,1)
            if(spot==1) color = 'r';
            elseif spot==2 color = 'b';
            elseif spot==3 color = 'k';
            elseif spot==4 color = 'g';
            elseif spot==5 color = 'm';
            elseif spot==6 color = 'y';
                % elseif spot==1 color = '.b';
            end
            
            scatter3( next_ee_wrt_RCM(1),next_ee_wrt_RCM(2),next_ee_wrt_RCM(3),'o',color)
            
            hold on
            grid on
            
            title('EE position')
            
            subplot(2,1,2)
            
            % plotting current position
            plot = scatter( [us_ee(1), us_ee(2), us_ee(3), us_ee(4)],...
                [vs_ee(1), vs_ee(2), vs_ee(3), vs_ee(4)], '*','k');
            
            % plotting desired position
            plot = scatter( [us_desired(1,spot), us_desired(2,spot), us_desired(3,spot), us_desired(4,spot) ],...
                [vs_desired(1,spot), vs_desired(2,spot), vs_desired(3,spot), vs_desired(4,spot)], 'r', 'o','filled');
            
            hold on
            grid on
            title('image error convergence')
                      
     
        end
               
        %____________________________________________________________________________________________________________________________________
        %____________________________________________________________________________________________________________________________________
        
    elseif mode==0
                
        % once inverse kinematics is done you can subs all this with inverse_kin
        % relativeToObjectHandle = h_RCM defined upward
        [~, ee_position_relative]=vrep.simxGetObjectPosition(ID, h_j6, relativeToObjectHandle, vrep.simx_opmode_streaming);
        [~, ee_orientation_relative]=vrep.simxGetObjectOrientation(ID, h_j6, relativeToObjectHandle, vrep.simx_opmode_streaming);
        ee_pose_relative = [ee_position_relative,ee_orientation_relative];
        sync=false;
        % getting the current pose
        
        while ~sync
            [~, ee_position_VS]=vrep.simxGetObjectPosition(ID, h_EE, -1, vrep.simx_opmode_streaming);
            sync = norm(ee_position_VS,2)~=0;
        end
        sync=false;
        %
        while ~sync
            [~, ee_orientation_VS]=vrep.simxGetObjectOrientation(ID, h_EE, -1, vrep.simx_opmode_streaming);
            sync = norm(ee_orientation_VS,2)~=0;
        end
        sync=false;
        %
        ee_pose_VS= [ee_position_VS, ee_orientation_VS]'; 
        %
        
        %computing the error
        err=[ee_pose_d(1:3) - ee_pose_VS(1:3); angdiff(ee_pose_VS(4:6), ee_pose_d(4:6)) ];
        %
        
        % evaluating exit condition
        if max(err)<=0.001
           mode=1;
           spot=spot+1;
           fprintf(1,'GOING TOWARD LANDMARK: %d \n',spot);
                      
           pause(1);
           time = 0;
           continue;
        end
              
        % computing the displacement
        ee_displacement = H*err;
        
        % updating the pose
        ee_pose_VS= ee_pose_VS + ee_displacement;
        [~]= vrep.simxSetObjectPosition(ID, h_EE, -1, ee_pose_VS(1:3), vrep.simx_opmode_oneshot);
        [~]= vrep.simxSetObjectOrientation(ID, h_EE, -1, ee_pose_VS(4:6), vrep.simx_opmode_oneshot);
          
    end
        
    pause(0.05);
end

fprintf(2,'**** PROCESS ENDED ***** \n');