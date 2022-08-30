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


startPoints = [0, 0, 0]; %home values
robot.servo_jp(startPoints); % send to home position
pause(1);

interpolateTime = 1000; % time of movement in ms
currentTime = 0;        % elapsed time
newTime = 0;

setPoints = [60, 60, 60]; % movements of whole arm

rmsData = zeros(3, 10);
errorVector = zeros(1,10);
%ideal
idealT = robot.fk3001(transpose(startPoints));
idealPos = [idealT(1,4),idealT(2,4),idealT(3,4)];

for t = 1:10
    
    robot.servo_jp(setPoints);
    robot.interpolate_jp(startPoints, interpolateTime);
    
    goalPos = robot.goal_js();
    tMatrix = zeros(4, 4); %transformation matrix from base to tip
    errorDeg = 2; % prevents undershooting
    
    tic % start timer (when does this start?)
    i = 1; %counter
    loopBool = false;
    
    while loopBool == false
        
        loopBool = robot.atGoalPos(errorDeg);
        tMatrix = robot.measured_cp(); % assign current position   
    end
    
    robot.servo_jp(setPoints);
    pause(1);
    xPos = tMatrix(1,4);
    yPos = tMatrix(2,4);
    zPos = tMatrix(3,4);
    
    rmsData(1, t) = xPos;
    rmsData(2, t) = yPos;
    rmsData(3, t) = zPos;
    error = rms(idealPos) - rms(rmsData(:,t));
    errorVector(1, t) = error;
    tipPos0 = plot3(xPos,yPos,zPos, '-o','MarkerSize',5); %tip positions at home 
    hold on;
   
end


hold off
title('tip position');
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
disp(tipPos0);
disp('average error');
disp(mean(errorVector));
robot.shutdown()