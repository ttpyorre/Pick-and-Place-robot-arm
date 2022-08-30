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

% Initialize classes.
robot = Robot(myHIDSimplePacketComs); 
model = Model(robot);

startPoints = [0, 0, 0]; %home values
robot.servo_jp(startPoints); % send to home position
pause(1);

while 1 
    angles = robot.measured_js(true, false);
    %disp(angles(1, :));
    model.FKdraw(angles(1, :)');
    pause(0.1);
end

robot.shutdown();




