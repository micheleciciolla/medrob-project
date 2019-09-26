%%
%   INIT STUFF
%%
cd(fileparts(mfilename('fullpath')));
clear all;
close all;
clc;

pause(3);
%%
% CONNECTION TO VREP
%%
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

% For sync purposes (see below)
ok=false; 

%%
% COLLECTING HANDLES
%%

% vision sensor
[~, h_VS]=vrep.simxGetObjectHandle(clientID, 'Vision_sensor_ECM', vrep.simx_opmode_blocking);

% force sensor
[~, h_FS]=vrep.simxGetObjectHandle(clientID, 'Force_sensor', vrep.simx_opmode_blocking);

% end effector
[~, h_EE]=vrep.simxGetObjectHandle(clientID, 'FollowedDummy', vrep.simx_opmode_blocking);

% landmarks
for k=1:4
    for h=1:5
        [~, h_L(k,h)]=vrep.simxGetObjectHandle(clientID, ['Landmark', num2str(h), num2str(k)], vrep.simx_opmode_blocking);
    end
end

% EE landmark
for k=1:4
    [~, h_L_EE(k)]=vrep.simxGetObjectHandle(clientID, ['LandmarkEE', num2str(k)], vrep.simx_opmode_blocking);
end

%%
%   SETTINGS
%%

%landmarks colors (old)
%grays=[0.8; 0.6; 0.4; 0.2]*255;     %landmarks' gray shades

%focal length (depth of the near clipping plane)
fl=0.01;

% control gain in mode 0 (see below)
K = eye(6)*(10^-3);

%control gain in mode 1 (see below)
H = eye(6)*(10^-1);

% compliance matrix
L = eye(6)*(10);

% desired features
for k=1:4
    for h=1:5
        %
        while ~ok
            [~, l_position]=vrep.simxGetObjectPosition(clientID, h_L(k,h), h_VS, vrep.simx_opmode_streaming);
            ok = norm(l_position,2)~=0;
        end
        ok=false;
        %
        us_d(k,h)= fl*l_position(1)/l_position(3);
        vs_d(k,h)= fl*l_position(2)/l_position(3);
        %
    end
end

% desired force and torque
force_torque_d=zeros(6,1);

% end effector home pose
ee_pose_d=[ -1.5413e+0;   -4.0699e-2;    +7.2534e-1;  -1.80e+2;         0;         0];

%%
%	PROCESS LOOP
%%

% two possible control modes:
    % mode 0: go-to-home control mode; 
    % mode 1: visual servoing eye-on-hand control mode;
mode=0

% mode 0 overview:
	% i) features and depth extraction

	% ii) building image jacobian and computing the error (vision-based only)
			
	% iii) adjusting the error (via the force-based infos)
	
    % iv) computing the ee displacement	
    
	% v) updating the pose

% mode 1 is just a Cartesian proportionale regulator

%start from landmark h+1
h=1-1;
%

% loop
while h<6
    
    if mode==1
        
        %%
        %	I) FEATURES and DEPTH EXTRACTION
        %%

        us=zeros(4,1);
        vs=zeros(4,1);
        zs=zeros(4,1);
        for k=1:4
            %
            while ~ok
                [~, l_position]=vrep.simxGetObjectPosition(clientID, h_L_EE(k), h_VS, vrep.simx_opmode_streaming);
                ok = norm(l_position,2)~=0;
            end
            ok=false;
            %
            zs(k)= l_position(3);
            us(k)= fl*l_position(1)/l_position(3);
            vs(k)= fl*l_position(2)/l_position(3);
            
        end
        
        
        %	II) BUILDING the IMAGE JACOBIAN and COMPUTING THE ERROR (vision-based only)
        %%
        
        %building the jacobian
        J = [ build_point_jacobian(us(1),vs(1),zs(1),fl); ...
              build_point_jacobian(us(2),vs(2),zs(2),fl); ...
              build_point_jacobian(us(3),vs(3),zs(3),fl); ...
              build_point_jacobian(us(4),vs(4),zs(4),fl)]; 
        %
        
        % computing the error
        err= [us_d(1,h)-us(1); ...
              vs_d(1,h)-vs(1); ...
              us_d(2,h)-us(2); ...
              vs_d(2,h)-vs(2); ...
              us_d(3,h)-us(3); ...
              vs_d(3,h)-vs(3); ...
              us_d(4,h)-us(4); ...
              vs_d(4,h)-vs(4)];
        %
        %norm(err,2)
        
        % evaluating exit condition
        if norm(err,2)<=10^-4
           if h==5
               break;
           end
           mode=0
           pause(1);
           continue;
        end
        %
        
        %%
        %	III) ADJUSTING the ERROR (via the force-based infos)
        %
        while ~ok
            [~, ~, force, torque]=vrep.simxReadForceSensor(clientID, h_FS, vrep.simx_opmode_streaming);
            ok = true; %norm(force,2)~=0;
        end
        ok=false;
        %
        force_torque=[force'; torque'];
        force_torque=round(force_torque,2);
        %
        err=err + J*L*(force_torque_d-force_torque);
        %
        
        %%
        %	IV) COMPUTING the EE DISPLACEMENT
        %%
        
        %
        while ~ok
            [~, abg]=vrep.simxGetObjectOrientation(clientID, h_VS, h_EE, vrep.simx_opmode_streaming);
            ok = norm(abg,2)~=0;
        end
        ok=false;
        %
        while ~ok
            [~, tr]=vrep.simxGetObjectPosition(clientID, h_VS, h_EE, vrep.simx_opmode_streaming);
            ok = norm(tr,2)~=0;
        end
        ok=false;
        %
        
        %computing the displacement
        ee_displacement = K*pinv(-J)*err;
        if norm(ee_displacement,2)<10^-2.5 %10^-2.9
            ee_displacement = (ee_displacement/norm(ee_displacement,2))*10^-2.5;
        end
        %
        
        %%
        %	V) UPDATING THE POSE
        %%

        % getting the current pose
        %
        while ~ok
            [~, ee_position]=vrep.simxGetObjectPosition(clientID, h_EE, h_VS, vrep.simx_opmode_streaming);
            ok = norm(ee_position,2)~=0;
        end
        ok=false;
        %
        while ~ok
            [~, ee_orientation]=vrep.simxGetObjectOrientation(clientID, h_EE, h_VS, vrep.simx_opmode_streaming);
            ok = norm(ee_orientation,2)~=0;
        end
        ok=false;
        %
        ee_pose= [ee_position, ee_orientation]'; 
        %
        
        %updating the pose
        ee_pose= ee_pose + ee_displacement;
        [~]= vrep.simxSetObjectPosition(clientID, h_EE, h_VS, ee_pose(1:3), vrep.simx_opmode_oneshot);
        [~]= vrep.simxSetObjectOrientation(clientID, h_EE, h_VS, ee_pose(4:6), vrep.simx_opmode_oneshot);
        % 
        
    elseif mode==0
        
        % getting the current pose
        %
        while ~ok
            [~, ee_position]=vrep.simxGetObjectPosition(clientID, h_EE, -1, vrep.simx_opmode_streaming);
            ok = norm(ee_position,2)~=0;
        end
        ok=false;
        %
        while ~ok
            [~, ee_orientation]=vrep.simxGetObjectOrientation(clientID, h_EE, -1, vrep.simx_opmode_streaming);
            ok = norm(ee_orientation,2)~=0;
        end
        ok=false;
        %
        ee_pose= [ee_position, ee_orientation]'; 
        %
        
        %computing the error
        err=[ee_pose_d(1:3) - ee_pose(1:3); angdiff(ee_pose(4:6), ee_pose_d(4:6)) ];
        %
        
        % evaluating exit condition
        if max(err)<=0.001
           mode=1
           h=h+1;
           fprintf(1,'GOING TOWARD LANDMARK: %d \n',h);
           
           pause(1);
           continue;
        end
        %
        
        %computing the displacement
        ee_displacement = H*err;
        %

        %updating the pose
        ee_pose= ee_pose + ee_displacement;
        [~]= vrep.simxSetObjectPosition(clientID, h_EE, -1, ee_pose(1:3), vrep.simx_opmode_oneshot);
        [~]= vrep.simxSetObjectOrientation(clientID, h_EE, -1, ee_pose(4:6), vrep.simx_opmode_oneshot);
        %   
    end
        
    pause(0.05);
end

%%
%	FUNCTIONS
%%
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