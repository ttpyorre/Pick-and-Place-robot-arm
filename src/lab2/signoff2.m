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

startPoints = [0 0 0];
robot.servo_jp(startPoints);
pause(1);
goToPoint = [0 60 0];
interpolateTime = 2000;
robot.interpolate_jp(goToPoint, interpolateTime);

j = robot.measured_cp();
disp("measure_cp: ");
disp(j);

h = robot.setpoint_cp();
disp("setpoint_cp: ");
disp(h);

g = robot.goal_cp();
disp("goal_cp: ");
disp(g);
pause(2.5);

j = robot.measured_cp();
disp("measure_cp: ");
disp(j);

h = robot.setpoint_cp();
disp("setpoint_cp: ");
disp(h);

g = robot.goal_cp();
disp("goal_cp: ");
disp(g);

robot.shutdown();
