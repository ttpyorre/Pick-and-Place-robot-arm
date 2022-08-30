
% RBE3001 - Laboratory 3
% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
traj_planner = Traj_Planner();
robot = Robot(myHIDSimplePacketComs); 
model = Model(robot);

positions = [150,45,67;
             30,104,210;
             100,-10,195;
             150,45,67];

startingPoint = robot.ik3001(positions(1,:));
robot.interpolate_jp(startingPoint, 2000);
pause(3);

errorDeg = 3;

trajTime = 1.5;
PV = [];
writematrix(PV,'Data.csv');

elapsed = 0;
tic;
for i = 2:4     
%     currentJoints = robot.ik3001(positions(i-1,:));
%     goalJoints = robot.ik3001(positions(i,:));
     
    currentJoints = positions(i-1,:);
    goalJoints = positions(i,:);
    coeff1 = traj_planner.cubic_traj(0,trajTime,0,0,currentJoints(1),goalJoints(1));
    coeff2 = traj_planner.cubic_traj(0,trajTime,0,0,currentJoints(2),goalJoints(2));
    coeff3 = traj_planner.cubic_traj(0,trajTime,0,0,currentJoints(3),goalJoints(3));
    
    coeffMatrix = [coeff1 coeff2 coeff3];
    
    currTime = toc + elapsed;
    PV = robot.run_trajectory(transpose(coeffMatrix), trajTime, true);
    PV(:,7) = PV(:,7) + currTime;
    writematrix(PV,'Data.csv','WriteMode','append');
    elapsed = currTime;
    
    pause(2);
end  

pause(2);

plotData = readmatrix('Data.csv');

figure(1)
plot(plotData(:,7),plotData(:,1),'LineWidth', 2);
hold on
plot(plotData(:,7),plotData(:,2),'LineWidth', 2);
plot(plotData(:,7),plotData(:,3),'LineWidth', 2);
hold off
title('Joint Space: Trajectory Position(deg) vs time(s)');
xlabel('Time(s)');
ylabel('Position (deg)');
legend('x position', 'y position', 'z position');

figure(2)
plot(plotData(:,7),plotData(:,4),'LineWidth', 2);
hold on
plot(plotData(:,7),plotData(:,5),'LineWidth', 2);
plot(plotData(:,7),plotData(:,6),'LineWidth', 2);
hold off
title('Joint Space: Trajectory Velocity(deg/s) vs time(s)');
xlabel('Time(s)');
ylabel('Velocity (deg/s)');
legend('x velocity', 'y velocity', 'z velocity');

dx = gradient(plotData(:,4)) ./ gradient(plotData(:,7));
dy = gradient(plotData(:,5)) ./ gradient(plotData(:,7));
dz = gradient(plotData(:,6)) ./ gradient(plotData(:,7));

figure(3)
plot(plotData(:,7),dx,'LineWidth', 2);
hold on
plot(plotData(:,7),dy,'LineWidth', 2);
plot(plotData(:,7),dz,'LineWidth', 2);
hold off
title('Joint Space: Trajectory Acceleration(deg/s^2) vs time(s)');
xlabel('Time(s)');
ylabel('Acceleration (deg/s^2)');
legend('x acceleration', 'y acceleration', 'z acceleration');

robot.shutdown();