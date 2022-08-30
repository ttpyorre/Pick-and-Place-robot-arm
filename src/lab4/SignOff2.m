
% RBE3001 - Laboratory 4
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

robot.interpolate_jp([0 0 0], 3000);
pause(5);

% Program starts
positions = [150,45,67;
             30,104,210;
             100,-10,195;
             150,45,67];

startingPoint = robot.ik3001(positions(1,:));


robot.interpolate_jp(startingPoint, 3000);

pause(5);

errorDeg = 2;
trajTime = 2;
PV = [];

writematrix(PV,'Data.csv');

elapsed = 0;
tic;
for i = 2:4         
    currentJoints = positions(i-1,:);
    goalJoints = positions(i,:);
    coeff1 = traj_planner.quintic_traj(0,trajTime, 0,0, currentJoints(1),goalJoints(1), 0, 0);
    coeff2 = traj_planner.quintic_traj(0,trajTime, 0,0, currentJoints(2),goalJoints(2), 0, 0);
    coeff3 = traj_planner.quintic_traj(0,trajTime, 0,0, currentJoints(3),goalJoints(3), 0, 0);
    
    coeffMatrix = [coeff1 coeff2 coeff3];
    currTime = toc + elapsed;
    newPV = robot.run_trajectory(transpose(coeffMatrix), trajTime, true);
    newPV(:,7) = newPV(:,7) + currTime;
    
    PV = [PV;newPV];
    
    elapsed = currTime;
    
    pause(2);
end  
writematrix(PV,'Data.csv');
pause(0.05);
plotData = readmatrix('Data.csv');

sz = size(plotData);
nRow = sz(1);
nCol = sz(2);

figure(2)
plot(plotData(2:nRow,7), plotData(2:nRow,1), 'LineWidth', 2);
hold on
plot(plotData(2:nRow,7), plotData(2:nRow,2), 'LineWidth', 2);
hold on
plot(plotData(2:nRow,7), plotData(2:nRow,3), 'LineWidth', 2);
hold off
title('linear velocity(mm/s) vs time(s)');
xlabel('Time(s)');
ylabel('linear velocity (mm/s)');
legend('x direction', 'y direction', 'z direction');

figure(3)
plot(plotData(2:nRow,7),plotData(2:nRow,4),'LineWidth', 2);
hold on
plot(plotData(2:nRow,7),plotData(2:nRow,5),'LineWidth', 2);
hold on
plot(plotData(2:nRow,7),plotData(2:nRow,6),'LineWidth', 2);
hold off
title('angular Velocity(deg/s) vs time(s)');
xlabel('Time(s)');
ylabel('angular velocity deg/s)');
legend('x direction', 'y direction', 'z direction');

scalarSpeed = [];
for i = 2:nRow
    squares = plotData(i,1)^2 + plotData(i,2)^2 + plotData(i,3)^2;
    scalarSpeed(i-1) = sqrt(squares);
end

figure(4)
plot(plotData(2:nRow,7),scalarSpeed(:),'LineWidth', 2);
title('Scalar speed of linear velocity (mm/s^2) vs time(s)');
xlabel('Time(s)');
ylabel('Scalar speed of linear velocity(mm/s^2)');

% %plotting path followed by robot arm in task space
% figure(5);
% plot3(plotData(2:nRow,1),plotData(2:nRow,2),plotData(2:nRow,3), 'LineWidth', 2);
% title('Arm path 3D space')
% xlabel('x (mm)');
% ylabel('y (mm)');
% zlabel('z (mm)');
robot.shutdown();