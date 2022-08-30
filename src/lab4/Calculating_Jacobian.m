% RBE3001 - Laboratory 4
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
% In this script we are simply doing our Jacobian calculations, that we
% then use in the Robot.m script to create the Jacobian function.

syms J1 J2 J3

%position values from the forward kinematics matrix
x = simplify(100*cos((J1))*cos(((J2 - 90))) + 100*cos((pi*J1)/180)*cos((pi*(J2 - 90))/180)*cos((pi*(J3 + 90))/180) - 100*cos((pi*J1)/180)*sin((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180));
y = simplify(100*sin((J1))*cos(((J2 - 90))) + 100*sin((pi*J1)/180)*cos((pi*(J2 - 90))/180)*cos((pi*(J3 + 90))/180) - 100*sin((pi*J1)/180)*sin((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180));
z = simplify(95 - 100*cos((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180) - 100*cos((pi*(J3 + 90))/180)*sin((pi*(J2 - 90))/180) - 100*sin((pi*(J2 - 90))/180));

dx1 = diff(x,J1);
dy1 = diff(y,J1);
dz1 = diff(z,J1);

dx2 = diff(x,J2);
dy2 = diff(y,J2);
dz2 = diff(z,J2);

dx3 = diff(x,J3);
dy3 = diff(y,J3);
dz3 = diff(z,J3);

jUp = simplify([dx1,dx2,dx3;
     dy1,dy2,dy3;
     dz1,dz2,dz3]);
disp("Upper J");
disp(rad2deg(jUp));
 
T = robot.baseToTransform([J1;J2;J3]);
disp(T);
% zi of Transformation matricies from T01 to T03
jDown = simplify([T(1:3,3),T(1:3,7),T(1:3,11)]);
disp("Lower J");
disp(jDown);

J = [jUp;jDown];
disp("J");
disp(J);