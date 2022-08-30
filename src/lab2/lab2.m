%%
% RBE3001 - Laboratory 2
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

%  L0 = 55;
%  L1 = 40;
%  L2 = 100;
% L3 = 100;
% 
% syms J1 J2 J3
% dh = [0,L0,0,0;J1,L1,0,-90;J2-90,0,L2,0;J3+90,0,L3,0];
% disp(robot.dh2fk(dh));
            
            
            
 setpoints = [0,0,0];
% a = [pi,1,0,pi];
% disp(robot.dh2mat(a)); % single frame transformation matrix
% 
disp('fk3001');
disp(robot.fk3001([0,0,0]));
                   
%%%%



errorDeg = 1;
goalPos = robot.goal_js();
currentPos = robot.measured_js(true,false);
jointAngles = transpose(currentPos(1,:));
disp(robot.fk3001(jointAngles));
pause(1);


robot.interpolate_jp(setpoints,2000);
while goalPos(1) >= currentPos(1,1)  + errorDeg
  
    currentPos = robot.measured_js(true, false); % assign current position
end
pause(1);



%testing for lab 2 sign_off 2 functions
disp('measured_cp():');
disp(robot.measured_cp());
disp('setpoint_cp():');
disp(robot.setpoint_cp());
disp('goal_cp:');
disp(robot.goal_cp());
disp(plot3(2,2,2));


%Clear up memory upon termination
robot.shutdown()

% 
% while 1
%  currPos = robot.measured_js(true,false);
%  disp(robot.fk3001(transpose(currPos(1,:))));
% end