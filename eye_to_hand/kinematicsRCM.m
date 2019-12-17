classdef kinematicsRCM
    % from distributed/dVKinematics.cpp
    properties
        t = zeros(27); % data structure to save data
        J = zeros(6,6); % jacobian
    end
    methods (Static)
        function [pos] = direct_kinematics(Q)
            
            % computes position of EE wrt RCM given joints values
            % output is 3x1 vector
                       
            q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4); q5 = Q(5); q6 = Q(6);
             
            t(2) = sin(q1);
            t(3) = cos(q1);
            t(4) = cos(q4);
            t(5) = sin(q2);
            t(6) = sin(q4);
            t(7) = cos(q5);
            t(8) = t(3)*t(6);
            t(9) = t(2)*t(4)*t(5);
            t(10) = t(8)+t(9);
            t(11) = cos(q2);
            t(12) = sin(q5);
            t(25) = q3-1.56e-2;
            t(26) = t(2)*t(6);
            t(28) = t(3)*t(4)*t(5);
            t(27) = t(26)-t(28);
            
            % computing coordinates
            x = t(10) * t(12) * (-9.1e-3) + t(2) * t(7) * t(11) * 9.1e-3 + t(2) * t(11) * t(25);
            y = - t(5) * t(7) * 9.1e-3 - t(5) * t(25) - t(4) * t(11) * t(12) * 9.1e-3;
            z = t(12) * t(27) * (-9.1e-3)- t(3) * t(7) * t(11) * 9.1e-3 - t(3) * t(11) * t(25);
            
            % do i need [roll, pitch, yaw]?
            
            pos = [x,y,z]';
        end
        
        function [Q] = inverse_kinematics(Q, err)
            
            % given error and current configuration returns next
            % configuration to converge to desired position -> err=0
            
            J = kinematicsRCM.compute_jacobian(Q);
            J = pinv(J);
            v = [1 1 1 0.4 0.4 0.4]*10^-1; % we're correcting more the position that orientation (-60%)
            alfa = diag(v);
            
            % computing gradient method for inverse kinematics
            Q = Q' + alfa*J*(err);
        end
        
        function [J] = compute_jacobian(Q)

            % computes Jacobian of current configuration
            % formula of Jacobian has been taken from distributed/dVKinematics.cpp
            
            q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4); q5 = Q(5); q6 = Q(6);
            l = 0;
            J = zeros(6,6);
            
            t2 = cos(q1);
            t3 = cos(q4);
            t4 = sin(q1);
            t5 = sin(q2);
            t6 = sin(q4);
            t7 = sin(q5);
            t8 = t4*t6;
            t15 = t2*t3*t5;
            t9 = t8-t15;
            t10 = cos(q2);
            t11 = cos(q5);
            t12 = q3-1.56E-2;
            t13 = cos(q6);
            t14 = sin(q6);
            t16 = t7*t9*9.1E-3;
            t17 = t2*t10*t11*9.1E-3;
            t18 = t5*t11*9.1E-3;
            t19 = t3*t7*t10*9.1E-3;
            t20 = t7*t9;
            t21 = t2*t10*t11;
            t22 = t20+t21;
            t23 = t3*t4;
            t24 = t2*t5*t6;
            t25 = t23+t24;
            t35 = t13*t22;
            t36 = t14*t25;
            t26 = t35-t36;
            t27 = t5*t11;
            t28 = t3*t7*t10;
            t29 = t27+t28;
            t30 = t13*t29;
            t31 = t6*t10*t14;
            t32 = t30+t31;
            t33 = t18+t19;
            t34 = t16+t17;
            t37 = t2*t10*t12;
            t38 = t2*t6;
            t39 = t3*t4*t5;
            t40 = t38+t39;
            t41 = t4*t10*t11*9.1E-3;
            t42 = t7*t40;
            t49 = t4*t10*t11;
            t43 = t42-t49;
            t44 = t2*t3;
            t47 = t4*t5*t6;
            t45 = t44-t47;
            t50 = t13*t43;
            t51 = t14*t45;
            t46 = t50-t51;
            t54 = t7*t40*9.1E-3;
            t48 = t41-t54;
            t52 = t9*t11;
            t65 = t2*t7*t10;
            t53 = t52-t65;
            t55 = t4*t10*t12;
            t56 = t5*t12;
            t57 = t18+t19+t56;
            t58 = t5*t7;
            t64 = t3*t10*t11;
            t59 = t58-t64;
            t60 = t11*t40;
            t61 = t4*t7*t10;
            t62 = t60+t61;
            t63 = t4*t10;
            
            J(1,1) = t16+t17+t37+l*t26;
            J(1,2) = -t4*t57-l*t4*t32;
            J(1,3) = t63;
            J(1,4) = t5*t34+l*t5*t26-t2*t10*t33-l*t2*t10*t32;
            J(1,5) = -t25*t33-l*t25*t32-t6*t10*t34-l*t6*t10*t26;
            J(1,6) = -l*t26*t59-l*t32*t53;
                        
            J(2,1) = 0.0;
            J(2,2) = -t4*(t41+t55-t7*t40*9.1E-3)-t2*(t16+t17+t37)-l*t2*t26+l*t4*t46;
            J(2,3) = -t5;
            J(2,4) = t4*t10*t34-t2*t10*t48+l*t4*t10*t26+l*t2*t10*t46;
            J(2,5) = -t25*t48-t34*t45+l*t25*t46-l*t26*t45;
            J(2,6) = -l*t26*t62+l*t46*t53;
            
            J(3,1) = t41-t54+t55-l*t46;
            J(3,2) = t2*t57+l*t2*t32;
            J(3,3) = -t2*t10;
            J(3,4) = t5*(t41-t54)-l*t5*t46-t4*t10*t33-l*t4*t10*t32;
            J(3,5) = t33*t45+l*t32*t45-t6*t10*t48+l*t6*t10*t46;
            J(3,6) = l*t32*t62+l*t46*t59;
            
            J(4,1) = 0.0;
            J(4,2) = -t2;
            J(4,3) = 0.0;
            J(4,4) = t63;
            J(4,5) = -t44+t47;
            J(4,6) = -t60-t61;
            
            J(5,1) = -1.0;
            J(5,2) = 0.0;
            J(5,3) = 0.0;
            J(5,4) = -t5;
            J(5,5) = t6*t10;
            J(5,6) = t59;
            
            J(6,1) = 0.0;
            J(6,2) = -t4;
            J(6,3) = 0.0;
            J(6,4) = -t2*t10;
            J(6,5) = -t23-t24;
            J(6,6) = -t52+t65;         
            
        end
    end
end