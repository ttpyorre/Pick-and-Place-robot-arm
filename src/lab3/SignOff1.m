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

%signoff 1
while 1
        j = robot.measured_js(true,false);
        disp('actual');
        disp(j);
        t = robot.fk3001(j(1,:));
        pos = [t(1,4),t(2,4),t(3,4)];
%         disp('position');
%         disp(pos);
        pause(.5);
        disp('inverse');
        disp(robot.ik3001(pos));
end

robot.shutdown();