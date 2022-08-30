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
robot.servo_jp(startPoints); % send to home position
pause(1);

positions = [0 30 0;
             30 0 0;
             15 15 30;
             0  0  30;
             60 60 60];
interpolateTime = 3000;
model = Model(robot);

actualSetPoint = zeros(5, 3);

for i = 1:5
    angles = positions(i, :)';
    robot.interpolate_jp(angles, 3000);
    
    goalPos = robot.goal_js();
    errorDeg = 2; % prevents undershooting
    
    loopBool = false;
    while loopBool == false
        posData = robot.read(robot.SERVER_ID_READ_POS);
        curAngles = [posData(3); posData(5); posData(7)];
        model.FKdraw(curAngles);
        loopBool = robot.atGoalPos(errorDeg);
        pause(0.1);
    end
    
    actualSetPoint(i, :) = curAngles;
    
    disp(robot.fk3001(curAngles));
    pause(1);
end

robot.shutdown();