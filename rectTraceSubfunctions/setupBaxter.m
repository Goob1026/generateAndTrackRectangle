function [ robot,peripherals,ft_sensor,baxGrip ] = setupBaxter( disBodyAvoid,disCollAvoid,disConSafe,disGravComp,arm )
%% Establish connections
%connect to baxter
robot = RobotRaconteur.Connect('tcp://localhost:4545/BaxterJointServer/Baxter');
%connect to peripherals
peripherals = RobotRaconteur.Connect('tcp://localhost:46604/BaxterPeripheralServer/BaxterPeripherals');
%set robot to raw position control mode
robot.setControlMode(uint8(3));
%enable/disable desired peripherals
peripherals.suppressBodyAvoidance(arm,uint8(disBodyAvoid));
peripherals.suppressCollisionAvoidance(arm,uint8(disCollAvoid));
peripherals.suppressContactSafety(arm,uint8(disConSafe));
peripherals.suppressGravityCompensation(arm,uint8(disGravComp));
%connect to F/T sensor
ftsense = RobotRaconteur.Connect('tcp://192.168.1.134:5200/sensors.ati/ATImini45Host');
ft_sensor = ftsense.get_ft(0);
ft_sensor.startRecordingData('output2');
ft_sensor.bias();
ft_sensor.wrench;
% connect to the grippper
baxGrip = RobotRaconteur.Connect('tcp://192.168.1.134:6006/GripperController/gripcon');
pause(5);

end

