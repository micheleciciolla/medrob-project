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

spot_to_change= 3;

pause(3);
while true
    [return_code, spot_output, ~, ~, ~] = vrep.simxCallScriptFunction(ID, ['L_Prismatic_joint'] ,vrep.sim_scripttype_childscript(),'changeColorRed',[spot_to_change],[],[],[],vrep.simx_opmode_blocking);
    
    if return_code==0
        disp("SUCCESS");
    else disp("ERROR");
    end
    
    spot_output; % deve essere uguale spot_to_change perche nel metodo c'è solo un copy-paste
    
    pause(2);
    [return_code, spot_output, ~, ~, ~] = vrep.simxCallScriptFunction(ID, ['L_Prismatic_joint'] ,vrep.sim_scripttype_childscript(),'changeColorGreen',[spot_to_change],[],[],[],vrep.simx_opmode_blocking);
    pause(2);

end