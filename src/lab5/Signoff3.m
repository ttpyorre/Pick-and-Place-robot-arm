
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


imBall = cam.getImage();
% Convert the image to the HSV color space.
%imBall = rgb2hsv(i);

% Get the saturation channel.
%saturation = imBall(:, :, 2);

% Threshold the image
[x,y] = cam.createMaskYellow(imBall);


%figure, imshow(y);

%convert to grey scale
BW = rgb2gray( y );

figure, imshow(BW);

%fill in image
fillI = imfill(BW,'holes');
%figure, imshow(fillI);


%find circle and centroid
props = regionprops(fillI, 'Centroid');
imshow(fillI); 
hold on

[centers, radii] = imfindcircles(fillI,[6 30]);
centersStrong = centers(1,:);
radiiStrong = radii(1);
viscircles(centersStrong, radiiStrong,'EdgeColor','b');
plot(centers(1), centers(2), '+')
title('yellow');

hold off

[x,y] = cam.createMaskRed(imBall);


%figure, imshow(y);

%convert to grey scale
BW = rgb2gray( y );

figure, imshow(BW);

%fill in image
fillI = imfill(BW,'holes');
%figure, imshow(fillI);


%find circle and centroid
props = regionprops(fillI, 'Centroid');
imshow(fillI); 
hold on

[centers, radii] = imfindcircles(fillI,[6 30]);
centersStrong = centers(1,:);
radiiStrong = radii(1);
viscircles(centersStrong, radiiStrong,'EdgeColor','b');
plot(centers(1), centers(2), '+')
title('red');

hold off

[x,y] = cam.createMaskGreen(imBall);


%figure, imshow(y);

%convert to grey scale
BW = rgb2gray( y );

figure, imshow(BW);

%fill in image
fillI = imfill(BW,'holes');
%figure, imshow(fillI);


%find circle and centroid
props = regionprops(fillI, 'Centroid');
imshow(fillI); 
hold on

[centers, radii] = imfindcircles(fillI,[6 30]);
centersStrong = centers(1,:);
radiiStrong = radii(1);
viscircles(centersStrong, radiiStrong,'EdgeColor','b');
plot(centers(1), centers(2), '+')
title('green');

hold off


[x,y] = cam.createMaskOrange(imBall);


%figure, imshow(y);

%convert to grey scale
BW = rgb2gray( y );

figure, imshow(BW);

%fill in image
fillI = imfill(BW,'holes');
%figure, imshow(fillI);


%find circle and centroid
props = regionprops(fillI, 'Centroid');
imshow(fillI); 
hold on

[centers, radii] = imfindcircles(fillI,[6 30]);
centersStrong = centers(1,:);
radiiStrong = radii(1);
viscircles(centersStrong, radiiStrong,'EdgeColor','b');
plot(centers(1), centers(2), '+')
title('orange');

hold off




% Clear up memory upon termination
robot.shutdown()