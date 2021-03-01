function [q_lambda] = genStraightJointPath(robot,q0,qf,q_prime_min,q_prime_max,N)
%UNTITLED18 Summary of this function goes here
%   Detailed explanation goes here


% Inequality constraint (joint limit) parameters:
QPparams.c = 0.9;
QPparams.eta = 0.01;
QPparams.epsilon_in = 0.15;
QPparams.e = 0.002;


qlimit = [robot(1).limit.lower_joint_limit,...
    robot(1).limit.upper_joint_limit];
lambda = [0:1/(N-1):1];
options = optimoptions('quadprog','Display','off');

[R0T0, P0T0] = fwdkin(robot.kin,q0); 


dq = qf-q0;


% Solve QP Problem and Generate Joint Space Path
q_lambda = zeros(7,length(lambda)); q_lambda(:,1) = q0;
exitflag = zeros(1,length(lambda)); 
qprev = q0; Ptemp = P0T0; Rtemp = R0T0;
for k = 1:(length(lambda)-1)
    lb = q_prime_min;
    ub = q_prime_max;
    
    
   H = 2*eye(7);
    f = -2*dq;
    
    % Generate inequality constraints for joint limits
    h = zeros(14,1);
    h(1:7) = qprev - qlimit(:,1);
    h(8:14) = qlimit(:,2) - qprev;
    
    sigma = inequality_bound(h, QPparams.c, QPparams.eta, QPparams.epsilon_in, QPparams.e);
    dhdq = [eye(7); -eye(7)];
    
    A = -dhdq;
    b = -sigma;
    
    % solve QP problem
    [q_prime_temp,~,exitflag(k)] = quadprog(H,f,A,b,[],[],lb,ub,[]...
        ,options);
    
    q_prime_temp = q_prime_temp(1:7);
    % check exit flag - all elements should be 1
    if exitflag(k) ~= 1
        disp('Generation Error')
        return;
    end
    q_prime(:,k) = q_prime_temp; qprev = qprev + (1/(N-1))*q_prime_temp;
    q_lambda(:,k+1) = qprev;   
    [Rtemp, Ptemp] = fwdkin(robot.kin,qprev);
    P0T_lambda(:,k+1) = Ptemp;  R0T_lambda(:,:,k+1) = Rtemp;
    
end
q_prime(:,end+1) = zeros(7,1);


end

