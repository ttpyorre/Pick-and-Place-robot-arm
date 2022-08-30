
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
robot = Robot(myHIDSimplePacketComs); 
model = Model(robot);

robot.servo_jp([0 0 0]);

pause(2);
     
 positions = [150,45,67;
              30,104,210;
              100,-10,195;
              150,45,67];
   

robot.servo_jp(robot.ik3001(positions(1,:)));

pause(2);

errorDeg = 2;
dataMatrix = [];%column 1:3 joints, 4:6 position, 7 time
%writematrix(dataMatrix,'jointAnglesData.csv');

tMatrix = [];
%writematrix(tMatrix, 'posData.csv');
tic;
for i = 2:4     
    
    goalJoints = robot.ik3001(positions(i,:));
    robot.interpolate_jp(goalJoints,5000);
    loopBool = false; 
    while loopBool == false
        angles = robot.measured_js(true,false);
        pos = robot.measured_cp();
        tMatrix = [tMatrix; pos(1:3, 4)', toc];
        a = pos(1:3,4);
        disp(a');
        dataMatrix = [dataMatrix; angles(1,:),toc];
        
        % arm plot
        
        q = [angles(1,1), angles(1,2), angles(1,3)]';
        model.FKdraw(q);
        
        loopBool = robot.atGoalPos(errorDeg);
    end
    pause(2);
end  
writematrix(dataMatrix,'jointAnglesData.csv');
writematrix(tMatrix, 'posData.csv');

sz = size(tMatrix);

%plotting path followed by robot arm in task space
figure(2);
plot3(tMatrix(:,1),tMatrix(:,2),tMatrix(:,3), 'LineWidth', 2);
title('Arm path 3D space')
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');
    
%plotting theta1 theta2 theta3 against time
ypts1 = dataMatrix(:,1);
ypts2 = dataMatrix(:,2);
ypts3 = dataMatrix(:,3);
xpts1 = dataMatrix(:,4);
figure(3);
plot(xpts1, ypts1);

hold on;
plot(xpts1,ypts2);

hold on;
plot(xpts1,ypts3);
hold off;

ylabel('angle (degree)');
xlabel('time(seconds)');
legend('theta 1','theta 2','theta 3');


%plotting x,y,z position against time
ypts4 = tMatrix(:,1);
ypts5 = tMatrix(:,2);
ypts6 = tMatrix(:,3);

xpts2 = tMatrix(:,4);
figure(4)
plot(xpts2, ypts4);
hold on

plot(xpts2,ypts5);
hold on;

plot(xpts2,ypts6);
hold off;

xlabel('time(seconds)');
ylabel('position(mm)');
legend('x position','y position', 'z position');


%Clear up memory upon termination
robot.shutdown()