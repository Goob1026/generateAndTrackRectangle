%% robot = RobotRaconteur.Connect('tcp://localhost:4545/BaxterJointServer/Baxter');robot.setControlMode(uint8(1));
%% Initial setup
clear;clc;
pause(10);
warning off;
%path to needed files
path1 = '/home/cats/Documents/oakesk/02_10_2021_rawPositionMode/communalFiles/';
addpath(path1)
addpath([path1,'longMoveRawPositionMode'])
addpath('/home/cats/Documents/oakesk/02_10_2021_rawPositionMode/controllerTesting/trace_square_subfunctions')

%% things that may be modified
%controller type:openLoop, optimBW_noLoad, optimBW_2lbLoad, optimBW_bothModels
cont = 'optimBW_bothModels';
tscale = 6; %tscale>1 will move slower
rectVer = 'rectV1'; %rectV1, rectV2, rectV3, rectLockQ2, rectNoLockQ2
loadStatus = 'weight'; %noGrip, grip, weight
arm = 'left';
interPoseNum = 1; %pose moved to before the initial of the desired path
numLoops = 3; %number of times to go around rectangle
NL = 100;%used for long movements
dtL = 0.02;%used for long movements
dt = 0.02; %control loop rate

%% generate the controller
control = genController_rawPositionMode_allData(cont);

%% generate path/trajectory
[c,t_interp,x_interp] = setupPathTraj(path1,rectVer,arm,tscale);

%% gen inter pose and bias
interPosesChoice = [zeros(7,1),[-55;-45;0;90;0;-45;0],[0;-35;zeros(5,1)]]*pi/180;
interPose = interPosesChoice(:,interPoseNum);
if strcmp(arm,'right')
    interPose = [-1;1;-1;1;-1;1;-1].*interPose;
end
qbias = [-0.85;18.1;-1.7;11.9;-0.15;6.9;-0.3]*pi/180;
if strcmp(arm,'right')
    qbias = [-1;1;-1;1;-1;1;-1].*qbias;
end

%% additional setup variables
%disable peripherals (1 to disable, 0 to have enabled)
disCollAvoid = 1; %collision avoidance
disConSafe = 1; %contact safety
disBodyAvoid = 1; %body avoidance
disGravComp = 1; %gravity compensation

%sound effect to tell when done
seff = load('gong.mat');

%% Establish connections
[robot,peripherals,ft_sensor,baxGrip] = setupBaxter(disBodyAvoid,disCollAvoid,disConSafe,disGravComp,arm );

%% setup variables
allData = genVariables(40000);
if strcmp(arm,'left')
    inds = 1:7;
else
    inds = 8:14;
end

%% GET TO INITIAL POSE
[~,~,~] = longMove(robot,interPose,NL,dtL,arm,qbias);
pause(5);
qinit = c{1}.q_lambda(:,1);
[~,~,~] = longMove(robot,qinit,NL,dtL,arm,qbias);
pause(5);

%% if with load, use buttons for grasping
baxGrip.setPosition(uint8(0),uint8(100),uint8(1));
pause(5);
if strcmp(loadStatus,'weight')
    tmp = peripherals.getNavigatorState(arm);
    disp('waiting for ok button press to close gripper')
    while tmp.ok_button == 0
        tmp = peripherals.getNavigatorState(arm);
    end
    baxGrip.setPosition(uint8(255),uint8(1),uint8(1));
    disp('waiting for cancel button press to start movement')
    while tmp.cancel_button == 0
        tmp = peripherals.getNavigatorState(arm);
    end
end

%uiwait(msgbox('at start pose? -wait until baxter is done moving to press ok'));
q0init = robot.joint_positions;

%%
pause(10);

tstartall = tic;

count = 0;
idxseg1 = zeros(1,numLoops); idxseg2 = zeros(1,numLoops); idxseg3 = zeros(1,numLoops); idxseg4 = zeros(1,numLoops); idxseg5 = zeros(1,numLoops);
for numberLoops = 1:numLoops
    %% SEGMENT 1
    lam = 0; segNum = 1;
    [ allData,count ] = executeSegment(robot,peripherals,ft_sensor,arm,dt,qbias,allData,lam,t_interp,x_interp,count,c,segNum,tstartall,inds,control );
    idxseg1(numberLoops) = count;
    
    %% SEGMENT 2
    lam = 0; segNum = 2;
    [ allData,count ] = executeSegment(robot,peripherals,ft_sensor,arm,dt,qbias,allData,lam,t_interp,x_interp,count,c,segNum,tstartall,inds,control );
    idxseg2(numberLoops) = count;
    
    %% SEGMENT 3
    lam = 0; segNum = 3;
    [ allData,count ] = executeSegment(robot,peripherals,ft_sensor,arm,dt,qbias,allData,lam,t_interp,x_interp,count,c,segNum,tstartall,inds,control );
    idxseg3(numberLoops) = count;
    
    %% SEGMENT 4
    lam = 0; segNum = 4;
    [ allData,count ] = executeSegment(robot,peripherals,ft_sensor,arm,dt,qbias,allData,lam,t_interp,x_interp,count,c,segNum,tstartall,inds,control );
    idxseg4(numberLoops) = count;
    
    %% Loop Transition (movement b/c start/end joint poses don't match
    lam = 0; segNum = 5;
    [allData,count] = executeSegment(robot,peripherals,ft_sensor,arm,dt,qbias,allData,lam,t_interp,x_interp,count,c,segNum,tstartall,inds,control);
    idxseg5(numberLoops) = count;
end
%% End
%switch back to velocity control mode
robot.setControlMode(uint8(1));

toc(tstartall)

allData = trimData(allData);
[errData,spatialData] = genErrors_Plots( allData,arm,inds,idxseg1,idxseg2,idxseg3,idxseg4,idxseg5,numLoops);
note = 'angles for eul and rotenorm are 1-yaw,2-pitch,3-roll, units is radians';
%% save data
if strcmp(cont,'openLoop')
    contStat = 'openLoop';
elseif strcmp(cont,'optimBW_noLoad')
    contStat = 'optimNoLoad';
elseif strcmp(cont,'optimBW_2lbLoad')
    contStat = 'optim2lbLoad';
elseif strcmp(cont,'optimBW_bothModels')
    contStat = 'optimBoth';
end
if strcmp(loadStatus,'grip')
    loadStat = 'noLoad';
elseif strcmp(loadStatus,'weight')
    loadStat = '2lbLoad';
end
%generate directory if needed
mkdir([pwd,'/traceRectangle/',datestr(now,'dd-mmm-yyyy'),'/Looped/',rectVer,'/',contStat,'_',loadStat])
save([pwd,'/traceRectangle/',datestr(now,'dd-mmm-yyyy'),'/Looped/',rectVer,'/',contStat,'_',loadStat,'/',rectVer,'_',cont,'_withLoad',loadStatus,'_tscale',num2str(tscale),'_arm',arm,'_Loops_',datestr(now,'dd-mmm-yyyy-HH-MM'),'.mat']);
sound(seff.y)
%% open gripper if needed
tmp = peripherals.getNavigatorState(arm);
if strcmp(loadStatus,'weight')
    disp('waiting for cancel button press to open gripper')
    while tmp.cancel_button == 0
        tmp = peripherals.getNavigatorState(arm);
    end
    baxGrip.setPosition(uint8(0),uint8(255),uint8(1));
end

%%
rmpath(path1)
rmpath([path1,'longMoveRawPositionMode'])
rmpath('/home/cats/Documents/oakesk/02_10_2021_rawPositionMode/controllerTesting/trace_square_subfunctions')


