path1 = '/home/cats/Documents/oakesk/02_10_2021_rawPositionMode/communalFiles/';
rectVer = 3;

load([path1,'rectV',num2str(rectVer),'/Segment1.mat'])
qf = q_lambda(:,1);
load([path1,'rectV',num2str(rectVer),'/Segment4.mat'])
q0 = q_lambda(:,end);

dt = 0.02; N = 100;
delqmax = [2;2;2;2;4;4;4]*0.3*dt*(N-1); 
q_prime_min = -delqmax;
q_prime_max = delqmax;

[robot_const,~] = defineBaxter();
addpath([path1,'longMoveRawPositionMode'])
lambda = [0:1/(N-1):1];
[q_lambda] = genStraightJointPath(robot_const(1),q0,qf,q_prime_min,q_prime_max,N);
rmpath([path1,'longMoveRawPositionMode'])
cd([path1,'rectV',num2str(rectVer)])
save('LoopTransition.mat','lambda','q_lambda')

for ii = 1:7
    subplot(3,3,ii);plot(0,q0(ii)*180/pi,'*',lambda,q_lambda(ii,:)*180/pi,1,qf(ii)*180/pi,'*');
end
