% INIT STUFF

cd(fileparts(mfilename('fullpath')));
clear ;
close all;
clc;

pause(3);

% CONNECTION TO VREP
[clientID , vrep] = init_connection();

% COLLECTING HANDLES
% vision sensor h_VS - force sensor h_FS - prox sensor h_PS - end effector h_EE
[h_VS , h_FS , h_PS , h_EE ] = collect_handles(clientID , vrep);

% COLLECTING LANDMARK HANDLES
f = 7; % number of foursome of landamrks
h_L = collect_landamrk_handle(clientID, f, vrep);


%%
%   SETTINGS
%%

% landmarks colors (old)
%grays=[0.8; 0.6; 0.4; 0.2]*255;     %landmarks' gray shades

% focal length (depth of the near clipping plane)
fl=0.001;

% control gain in mode 0 (see below)
K = eye(6)*(10^-3);

% control gain in mode 1 (see below)
H = eye(6)*(10^-1);

% new: mik
% compliance matrix
C = eye(6)*(10);

% desired features
% loop rimbalzi
% us_d=[ -1.4987; 1.5013; 1.4997; -1.5004]*0.001;
% vs_d=[ -1.5005; -1.4988; 1.5012; 1.4995]*0.001;
% un rimbalzo
us_d=[ -0.1959; 0.2057; 0.2056; -0.1960]*fl; % focal lenght
vs_d=[ -0.2028; -0.2027; 0.1989; 0.1988]*fl;

% desired force and torque
force_torque_d=zeros(6,1);

% end effector home pose
ee_pose_d=[ -1.5387;   -0.1375;    0.7753;  pi;         0;         0];

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

% start from FIRST landmark
h = 0;

% loop
ok=false;

iterations = 3;

while h <= iterations
    % state: the detection state (false=no detection)
    % detectedPoint: the detected point coordinates (relative to the sensor reference frame)
    % detectedObjectHandle: handle of detected object
    % detectedSurfaceNormalVector: normalized normal vector of detected surface (relative to sensor reference frame)
    [~, state, detectedPoint,~,~ ]=vrep.simxReadProximitySensor(clientID, h_PS, vrep.simx_opmode_streaming);
    
    if mode==1
        
        %%
        %	I) FEATURES and DEPTH EXTRACTION
        %%
        
        us=zeros(4,1);
        vs=zeros(4,1);
        zs=zeros(4,1);
        
        for k=1:4
            while ~ok
                % EXTRACTING COORDINATES OF LANDMARK WRT VISION SENSOR
                [~, l_position]=vrep.simxGetObjectPosition(clientID, h_L(k,h), h_VS, vrep.simx_opmode_streaming);
                ok = norm(l_position,2)~=0;
            end
            ok=false;
            
            zs(k)= l_position(3);
            us(k)= fl*l_position(1)/l_position(3);
            vs(k)= fl*l_position(2)/l_position(3);
            
        end
        
        if mod(c,5)==0
            x=scatter(us,vs,'b');
            hold on;
        end
        c = c+1;
        
        %	II) BUILDING the IMAGE JACOBIAN and COMPUTING THE ERROR (vision-based only)
        
        % new: mik 
        % building the jacobian = INTERACTION MATRIX
        L = [ build_point_jacobian(us(1),vs(1),zs(1),fl); ...
            build_point_jacobian(us(2),vs(2),zs(2),fl); ...
            build_point_jacobian(us(3),vs(3),zs(3),fl); ...
            build_point_jacobian(us(4),vs(4),zs(4),fl)];
        
        % computing the error
        err= [us_d(1)-us(1); ...
            vs_d(1)-vs(1); ...
            us_d(2)-us(2); ...
            vs_d(2)-vs(2); ...
            us_d(3)-us(3); ...
            vs_d(3)-vs(3); ...
            us_d(4)-us(4); ...
            vs_d(4)-vs(4)];
        % norm(err,2)
        % evaluating exit condition
        if norm(err,2)<=10^-4
            if h==8
                break;
            end
            mode = 0;
            pause(1);
            disp("GOING HOME POSITION");
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
        
        % disp("force");
        % disp(force_torque(1:3));
        % disp("torque");
        % disp(torque);
                
        err= err - L*C*(force_torque_d-force_torque); % ALWAYS ZERO (force_torque_d-force_torque) / why minus - ?
        
        % PRINTING ERROR BETWEEN 
        if mod(c,5)==0 % c modulo 5 -> ritorna il resto (stampa ogni 5 cicli)
            x_d = scatter(err([1;3;5;7])+us,err([2;4;6;8])+vs,'r','filled');
            hold on;
        end
        c = c+1;
        
        %%
        %	IV) COMPUTING the EE DISPLACEMENT
        %%
        
        % computing the displacement
        ee_displacement = K*pinv(L)*err;
        if norm(ee_displacement,2)< 10^-2.9
            ee_displacement = (ee_displacement/norm(ee_displacement,2))*10^-2.9;
        end
                
        %%
        %	V) UPDATING THE POSE
        %%
        
        % getting the current pose and updating
        ee_pose= zeros(6,1) + ee_displacement;
        
        update_EE_pose(clientID , h_EE, ee_pose, vrep, mode);
           
        
    elseif mode==0
        %% proportional regulator
        
        % getting the current pose
        
        % NOT ENABLED - TODO
        % ee_pose = get_pose(clientID, h_EE, vrep); 
        
        while ~ok
            [~, ee_position]=vrep.simxGetObjectPosition(clientID, h_EE, -1, vrep.simx_opmode_streaming);
            ok = norm(ee_position,2)~=0;
        end
        ok=false;
        while ~ok
            [~, ee_orientation]=vrep.simxGetObjectOrientation(clientID, h_EE, -1, vrep.simx_opmode_streaming);
            ok = norm(ee_orientation,2)~=0;
        end
        ok=false;
        
        ee_pose= [ee_position, ee_orientation]';
        
        % computing the error
        err=[ee_pose_d(1:3) - ee_pose(1:3); angdiff(ee_pose(4:6), ee_pose_d(4:6)) ];
        
        % evaluating exit condition
        if max(err)<=0.1
            mode = 1;
            h=h+1;
            fprintf(1,'GOING TOWARD LANDMARK: %d \n',h);
            
            figure;
            title(['convergence toward landmark ', num2str(h)]);
            grid on;
            hold on;
            x_d=scatter(us_d,vs_d,'r','filled');
            xlabel('u');
            ylabel('v');
            c=0;
            
            pause(1);
            continue;
        end
        %
        
        % computing the displacement
        % H gain matrix eye(6)*10.^-1
        ee_displacement = H*err;
        
        % updating the pose
        ee_pose= ee_pose + ee_displacement;
        update_EE_pose(clientID , h_EE, ee_pose, vrep, mode)
    end
    
    pause(0.05);
    
end

disp("######## Process ended ########");


%%
%	FUNCTIONS
%%

% new : mik
function [] = update_EE_pose(clientID , h_EE, ee_pose, vrep, mode)

% SENDIND ORIENTATION AND POSITION to EE (only mode 1 )
if(mode == 1)
    [~]= vrep.simxSetObjectPosition(clientID, h_EE, h_EE, ee_pose(1:3), vrep.simx_opmode_oneshot);
    [~]= vrep.simxSetObjectOrientation(clientID, h_EE, h_EE, ee_pose(4:6), vrep.simx_opmode_oneshot);
    return
end
% SENDIND ORIENTATION AND POSITION to EE (only mode 0)
[~]= vrep.simxSetObjectPosition(clientID, h_EE, -1, ee_pose(1:3), vrep.simx_opmode_oneshot);
[~]= vrep.simxSetObjectOrientation(clientID, h_EE, -1, ee_pose(4:6), vrep.simx_opmode_oneshot);

end

% new : mik
function ee_pose = get_pose(clientID, h_EE, vrep)

% GETTING ABSOLUTE ORIENTATION AND POSITION OF EE

[~, ee_position]=vrep.simxGetObjectPosition(clientID, h_EE, -1, vrep.simx_opmode_streaming);
[~, ee_orientation]=vrep.simxGetObjectOrientation(clientID, h_EE, -1, vrep.simx_opmode_streaming);
ee_pose= [ee_position, ee_orientation]';

end

% new : mik
function [clientID , vrep] = init_connection()

% STABILIZING CONNECTION WITH VREP ENVIROMENT

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

% new : mik
function [h_VS, h_FS, h_PS, h_EE] = collect_handles(clientID, vrep)

% vision sensor
[~, h_VS]=vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_blocking);

% force sensor
[~, h_FS]=vrep.simxGetObjectHandle(clientID, 'Force_sensor', vrep.simx_opmode_blocking);

% prox sensor
[~, h_PS]=vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_blocking);

% end effector
[~, h_EE]=vrep.simxGetObjectHandle(clientID, 'FollowedDummy', vrep.simx_opmode_blocking);

end

% new : mik
function [h_L] = collect_landamrk_handle(clientID, f, vrep)
% COLLECTION OF HANDLES FOR EACH LANDMARK
for k=1:4
    for h=1:f
        [~, h_L(k,h)]=vrep.simxGetObjectHandle(clientID, ['Landmark', num2str(h), num2str(k)], vrep.simx_opmode_blocking);
    end
end
end


function [J] = build_point_jacobian(u,v,z,fl)
% THIS IS THE INTERACTION MATRIX [L]
%
J = [ -fl/z     0          u/z     (u*v)/fl        -(fl+(u^2)/fl)      v; ...
    0         -fl/z      v/z     (fl+(v^2)/fl)    -(u*v)/fl          -u];

end


% non usato -> la norma di detectedPoint è già la distanza
function [distance] = computeDistance(A,B)
distance = norm(sqrt(sum((A - B) .^ 2)));

end


%	OLD
%%

%{
    %getting the features
    if ~isempty(image)
        fs=extract_features(image, grays);
    end
%}

function [fs] = extract_features(image, grays)

fs=zeros(4,1);

rimage=image(:,:,1);
gimage=image(:,:,2);
bimage=image(:,:,3);

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