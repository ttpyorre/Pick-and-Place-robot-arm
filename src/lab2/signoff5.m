clear
clear java
clear   classes;

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
SERVER_ID_READ_POS = 1910; % ID of position packet

startPoints = [0, 0, 0]; %home values
pause(1);
model = Model(robot);


%%% Recording Positions %%%

%                   YZ:                   
% 1: 90,69.34,14.27
% 2: 90,49.9,-37.33
% 3: 90,-16.58,13.79
%                   XZ:
% 1: 0,71.31,10.27
% 2: 0, 41,-27.33
% 3: 0,-16.58,10.45

% YZpos = [90,49.9,-37.33;
%          90,-16.58,13.79;
%          90,69.34,14.27];

XZpos = [0, 41,-27.33;
         0,-16.58,10.45;
         0,71.31,10.27];
   
%robot.servo_jp([90,69.34,14.27;]);
robot.servo_jp([0,71.31,10.27]);
errorDeg = 2;
dataMatrix = []; %column 1:3 joints, 4:6 position, 7 time
writematrix(dataMatrix,'posData.csv');
tic; 
for i = 1:3     
    
    
    robot.interpolate_jp(XZpos(i,:),1000);
    loopBool = false; 
    
    while loopBool == false
        currPos = robot.measured_js(true,false);     
        tMatrix = robot.measured_cp();
        loopBool = robot.atGoalPos(errorDeg);
        dataMatrix(1, 1:3) = transpose(tMatrix(1:3,4));
        writematrix(dataMatrix,'posData.csv','WriteMode','append');
        dataMatrix = [];
        
        % arm plot
        
        q = [currPos(1,1), currPos(1,2), currPos(1,3)]';
        model.FKdraw(q);
    end
    pause(1);
    

end  

M = readmatrix('posData.csv');
    
    
    figure(1);
    plot(M(:,4),M(:,6), 'LineWidth', 2);
    title('XZ Positions')
    xlabel('y position (mm)');
    ylabel('z position (mm)');
    
    figure(2);
    plot(M(:,7),M(:,1:3), 'LineWidth', 2);
    title('Joint angles over time')
    xlabel('time (s)');
    ylabel('joint angle values (deg)');
    legend('x','y','z');


%Clear up memory upon termination
robot.shutdown()