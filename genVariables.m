function [ allData ] = genVariables( numInd )
%% set up variables
allData.t = zeros(1,numInd+1);             %time vector
allData.q_desired=zeros(7,numInd);       %desired joint positions
allData.ftdata = zeros(6,numInd);        %F/T sensor data
allData.q_baxter=zeros(14,numInd);       %measured joint positions
allData.q_vel_baxter = zeros(14,numInd); %measured joint velocities
allData.tor_baxter = zeros(14,numInd);   %measured joint torques
allData.q_error = zeros(7,numInd);       %computed joint error
allData.q_c = zeros(7,numInd);           %commanded joint positions
allData.q_ce = zeros(7,numInd);          %variable used for controller
allData.q_bias = zeros(7,numInd);        %feedforward for gravity compensation
allData.commQ = zeros(14,numInd);        %low-level joint position command
allData.commTor = zeros(14,numInd);      %command torque

end

