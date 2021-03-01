function [ allData,count ] = executeSegment(robot,peripherals,ft_sensor,arm,dt,qbias,allData,lam,t_interp,x_interp,count,c,segNum,tstartall,inds,control )
tmp = peripherals.getNavigatorState('left');
tstart1 = tic;
while (lam < 0.999999999) && (tmp.ok_button == 0)
    tic;
    %get current lam of t
    lam = interp1(t_interp,x_interp,toc(tstart1));
    count = count+1;
    %compute desired joint positions
    q_temp = interp1(0:0.001:1,c{segNum}.qs',lam)';
    allData.q_desired(:,count) = q_temp;
    %read data
    allData.q_baxter(:,count) = robot.joint_positions;
    allData.q_vel_baxter(:,count) = robot.joint_velocities;
    allData.commTor(:,count) = peripherals.commanded_torque;
    allData.commQ(:,count) = peripherals.commanded_position;
    allData.tor_baxter(:,count) = robot.joint_torques;
    allData.ftdata(:,count) = ft_sensor.wrench;
    %compute joint command from controller
    allData.q_error(:,count) = allData.q_desired(:,count)-allData.q_baxter(inds,count);
    allData.q_bias(:,count) = qbias;
    if count>4
        allData.q_ce(:,count) = eval(control);
%         if allData.q_ce(6,count) > 7*pi/180
%             allData.q_ce(6,count) = 7*pi/180;
%         elseif allData.q_ce(6,count) <-7*pi/180
%             allData.q_ce(6,count) = -7*pi/180;
%         end
        allData.q_c(:,count) = allData.q_desired(:,count)+allData.q_ce(:,count)-allData.q_bias(:,count);
    else
        allData.q_ce(:,count) = zeros(7,1);
        allData.q_c(:,count) = allData.q_desired(:,count)-allData.q_bias(:,count);
    end
    %send command to baxter
    robot.setJointCommand(arm,allData.q_c(:,count));
    %control loop rate
    allData.t(count+1) = toc(tstartall);
    try
        java.lang.Thread.sleep((dt-toc)*1000);
    catch ME
        pause(dt-toc);
    end
    tmp = peripherals.getNavigatorState('left');
end

end

