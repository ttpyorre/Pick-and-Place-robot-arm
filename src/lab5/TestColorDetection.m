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
load("cam1.mat");
robot = Robot(myHIDSimplePacketComs);
robot.openGripper();
pause(1);
findBall = true;

while findBall
    
home = [0,0,0];
robot.servo_jp(home);

try 
    [color, pos] = cam1.findColorBall();
catch
    robot.shutdown();
    findBall = false;
    error("no balls found");
end

aboveJoints =  robot.ik3001([pos, 50] ); %needs to be above ball
jointsAlign = aboveJoints + [-10,0,0];
posAlign = robot.fk3001(jointsAlign'); 
posLower = posAlign(1:3,4)' + [0,0,-35]; %lower to ball
jointsLower = robot.ik3001(posLower);

%%Finding color position
if strcmp(color,'red')
    sortPos = [0,100,35]; %top right

end
if strcmp(color,'yellow')
    sortPos = [0,-100,35]; %top left

end
if strcmp(color,'green')
    sortPos = [140,60,35]; %bottom right

end
if strcmp(color,'orange')
    sortPos = [140,-60,35]; %bottom left

end
if strcmp(color,'blue')
    sortPos = [140,0,32]; %bottom center

end


sortJoint = robot.ik3001(sortPos); %joint value of where its placed by color

posLowerPlace = sortPos + [0,0,-15]; %lower to ball

jointLowerPlace = robot.ik3001(posLowerPlace); %lower to place genty on board

%all joint config and time for loop
jointMatrix = [home;aboveJoints; jointsAlign;jointsLower;aboveJoints;sortJoint;jointLowerPlace;home]; 
timeMatrix = [1;2;1;1;1;2;1;3;2]; 


%loop
for j = 1:8

    currJoints = robot.measured_js(true,false);

    coeff1 = traj_planner.quintic_traj(0,timeMatrix(j),0,0,currJoints(1,1),jointMatrix(j,1),0,0);
    coeff2 = traj_planner.quintic_traj(0,timeMatrix(j),0,0,currJoints(1,2),jointMatrix(j,2),0,0);
    coeff3 = traj_planner.quintic_traj(0,timeMatrix(j),0,0,currJoints(1,3),jointMatrix(j,3),0,0);
    coeffMatrix = [coeff1 coeff2 coeff3];

    robot.run_trajectory(coeffMatrix', timeMatrix(j), false);

    if j == 4
        robot.closeGripper();
    end

    if j == 7
        robot.openGripper();
    end

    pause(1)
end

end



% Clear up memory upon termination
robot.shutdown()

