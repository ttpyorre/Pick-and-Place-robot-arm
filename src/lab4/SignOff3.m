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
traj_planner = Traj_Planner();
robot = Robot(myHIDSimplePacketComs);

startingPos = [50, 50, 0];
robot.servo_jp(startingPos);

pause(1);
positions = [0, 0, -90];
trajTime = 4;

coeff1 = traj_planner.quintic_traj(0,trajTime, 0,0, startingPos(1),positions(1), 0, 0);
coeff2 = traj_planner.quintic_traj(0,trajTime, 0,0, startingPos(2),positions(2), 0, 0);
coeff3 = traj_planner.quintic_traj(0,trajTime, 0,0, startingPos(3),positions(3), 0, 0);

coeffMatrix = [coeff1 coeff2 coeff3];

%code that actually gives plot


try
    newPV = robot.run_trajectory(coeffMatrix', trajTime, false);
    pause(2);
catch
    singluarityMotion = readmatrix('singluarityMotion.csv');
    
    figure(1);
    plot3(singluarityMotion(:,1), singluarityMotion(:,2), singluarityMotion(:,3));
    title('motion of robot arm in task space');
    xlabel('x position (mm)');
    ylabel('y position (mm)');
    zlabel('z position (mm)');
    
    figure(2);
    plot(singluarityMotion(:,5), singluarityMotion(:,4));
    xlabel('time (s)');
    ylabel('determinant of Upper Jacobian Matrix');
    title('Determinant of Upper Jacobian Matrix vs time');
end



%code that will only give error message
%newPV = robot.run_trajectory(coeffMatrix', trajTime, false);
%pause(2);

robot.shutdown();