
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
load("cam1.mat")
robot = Robot(myHIDSimplePacketComs);

imBall = cam1.getImage();
image = imBall;
imshow(image);
% Convert the image to the HSV color space.
%imBall = rgb2hsv(i);

% Get the saturation channel.
%saturation = imBall(:, :, 2);

% Threshold the image
[x,y] = cam1.createMaskBlue(imBall);


%figure, imshow(y);

%convert to grey scale
BW = rgb2gray( y );

figure, imshow(BW);

%fill in image
fillI = imfill(BW,'holes');
%figure, imshow(fillI);


%find circle and centroid
props = regionprops(fillI, 'Centroid');
%imshow(fillI); 
[centers, radii] = imfindcircles(fillI,[6 30]);
centersStrong = centers(1,:);
radiiStrong = radii(1);
viscircles(centersStrong, radiiStrong,'EdgeColor','b');

F0CH = [0, 1, 0, 50;
        1, 0, 0, -100;
        0, 0, -1, 0;
        0, 0, 0, 1];
  
    
%height of the ball in mm
ballHeight = 10;

%height of the camera in mm
camHeight = 110;

ratio = ballHeight/camHeight;

%distance from the origin of the checker board to the camera (measured
%along y-axis in mm
camToBoardLen = 350;

[centers(1) centers(2)]
pw = pointsToWorld(cam1.cam_IS, cam1.cam_pose(1:3,1:3),cam1.cam_pose(1:3,4),[centers(1) centers(2)])

%y coordinate needs to be fixed for visual error
pw = pw + (ratio * [125 - pw(1), camToBoardLen - pw(2)])

F0CH*[pw(1); pw(2); 0; 1]

% Clear up memory upon termination
robot.shutdown()