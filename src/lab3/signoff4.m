
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

% Program starts
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
    currentJoints = robot.ik3001(positions(i-1,:));
    goalJoints = robot.ik3001(positions(i,:));
    
     
%     currentJoints = positions(i-1,:);
%     goalJoints = positions(i,:);
    coeff1 = traj_planner.cubic_traj(0,trajTime,0,0,currentJoints(1),goalJoints(1));
    coeff2 = traj_planner.cubic_traj(0,trajTime,0,0,currentJoints(2),goalJoints(2));
    coeff3 = traj_planner.cubic_traj(0,trajTime,0,0,currentJoints(3),goalJoints(3));
    
    coeffMatrix = [coeff1 coeff2 coeff3];
    
    currTime = toc + elapsed;
    PV = robot.run_trajectory(transpose(coeffMatrix), trajTime, false);
    PV(:,4) = PV(:,4) + currTime;
    writematrix(PV,'Data.csv','WriteMode','append');
    elapsed = currTime;
    
    pause(2);
end  

pause(2);

plotData = readmatrix('Data.csv');

figure(1)
plot(plotData(:,4),plotData(:,1),'LineWidth', 2);
hold on
plot(plotData(:,4),plotData(:,2),'LineWidth', 2);
plot(plotData(:,4),plotData(:,3),'LineWidth', 2);
hold off
title('Cubic Task Space: Trajectory Position(mm) vs time(s)');
xlabel('Time(s)');
ylabel('position (mm)');
legend('x position', 'y position', 'z position');

dx = gradient(plotData(:,1)) ./ gradient(plotData(:,4));
dy = gradient(plotData(:,2)) ./ gradient(plotData(:,4));
dz = gradient(plotData(:,3)) ./ gradient(plotData(:,4));

figure(2)
plot(plotData(:,4),dx,'LineWidth', 2);
hold on
plot(plotData(:,4),dy,'LineWidth', 2);
plot(plotData(:,4),dz,'LineWidth', 2);
hold off
title('Cubic Task Space: Trajectory Velocity(mm/s) vs time(s)');
xlabel('Time(s)');
ylabel('velocity (mm/s)');
legend('x velocity', 'y velocity', 'z velocity');

d2x = gradient(dx) ./ gradient(plotData(:,4));
d2y = gradient(dy) ./ gradient(plotData(:,4));
d2z = gradient(dz) ./ gradient(plotData(:,4));

figure(3)
plot(plotData(:,4),d2x,'LineWidth', 2);
hold on
plot(plotData(:,4),d2y,'LineWidth', 2);
plot(plotData(:,4),d2z,'LineWidth', 2);
hold off
title('Cubic Task Space: Trajectory Acceleration(mm/s^2) vs time(s)');
xlabel('Time(s)');
ylabel('Acceleration (mm/s^2)');
legend('x acceleration', 'y acceleration', 'z acceleration');

robot.shutdown();