
% RBE3001 - Final Project
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
load("cam.mat")
robot = Robot(myHIDSimplePacketComs);

i = cam.getImage();
imshow(i);


%Results
F0CH = [0, 1, 0, 50;
        1, 0, 0, -100;
        0, 0, -1, 0;
        0, 0, 0, 1];
  

pw = pointsToWorld(cam.cam_IS, cam.cam_pose(1:3,1:3),cam.cam_pose(1:3,4),[173 81])


F0CH*[pw(1); pw(2); 0; 1]





% Clear up memory upon termination
robot.shutdown()