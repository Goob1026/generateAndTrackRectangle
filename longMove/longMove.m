function [iter,qall,tall] = longMove(robot,qf,N,dt,arm,qbias)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if strcmp(arm,'left')
    armNum = 1;
elseif strcmp(arm,'right')
    armNum = 2;
else
    disp('Incorrect Arm Definition');
    return;
end
delqmax = [2;2;2;2;4;4;4]*0.3*dt*(N-1); 

q_prime_min = -delqmax;
q_prime_max = delqmax;

[robot_const,~] = defineBaxter();
q0 = robot.joint_positions;
if armNum == 1
    q0 = q0(1:7);
else
    q0 = q0(8:end);
end
[q] = genStraightJointPath(robot_const(armNum),q0,qf,q_prime_min,q_prime_max,N);

t = 0:dt:(dt*(N-1));

qall = q; %%

for ii = 1:N
    qc = q(:,ii)-qbias;
    robot.setJointCommand(arm,qc);
    pause(dt)
end
iter = 1;
tall = t;
qprev = zeros(7,1);
while (norm(q(:,end)-qf)>1e-3) && (norm(qprev-q(:,end))>1e-3)
    iter = iter+1;
    qprev = q(:,end);
    [q] = genStraightJointPath(robot_const(armNum),q(:,end),qf,q_prime_min,q_prime_max,N);      
    tall = [tall,t+tall(end)+dt];
    qall = [qall,q];
    for ii = 1:N
        qc = q(:,ii)-qbias;
        robot.setJointCommand(arm,qc);
        pause(dt)
    end
    disp(['Iteration ',num2str(iter)])
end

end

