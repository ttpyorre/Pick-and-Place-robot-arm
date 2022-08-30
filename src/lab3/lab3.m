
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

positions = [150,45,67;
             30,104,210;
             100,-10,195];
   

robot.servo_jp([0,0,0]);
errorDeg = 3;
dataMatrix = []; %column 1:3 joints, 4:6 position, 7 time
writematrix(dataMatrix,'posData.csv'); 
for i = 1:3     
    
    goalJoints = robot.ik3001(positions(i,:));
    robot.interpolate_jp(goalJoints,1000);
    loopBool = false; 
    while loopBool == false
        currPos = robot.measured_js(true,false);     
        tMatrix = robot.measured_cp();
        dataMatrix(1,1:3) = currPos(1,:); 
        writematrix(dataMatrix,'posData.csv','WriteMode','append');
        dataMatrix = [];
        disp("Curr");
        disp(currPos);
%         disp("GJ");
%         disp(goalJoints(i,:));
        disp("RG");
        disp(robot.goal);
        
        % arm plot
        
        q = [currPos(1,1), currPos(1,2), currPos(1,3)]';
        model.FKdraw(q);
        
        loopBool = robot.atGoalPos(errorDeg);
    end
    pause(2);

end  

M = readmatrix('posData.csv');
   
figure(2);
plot3(M(:,1),M(:,2),M(:,3), 'LineWidth', 2);
title('Arm path 3D space')
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');
    


%Clear up memory upon termination
robot.shutdown()