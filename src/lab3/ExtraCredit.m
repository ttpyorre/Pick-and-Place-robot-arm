
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
traj_planner = Traj_Planner();
robot = Robot(myHIDSimplePacketComs); 
model = Model(robot);

robot.servo_jp([0 0 0]);

pause(1);
positions = [150,45,67;
             30,104,210;
             100,-10,195;
             150,45,67];
 goal = 1;        
         
for t = 1:3
    startingPoint = robot.ik3001(positions(1,:));
    robot.interpolate_jp(startingPoint, 3000);

    pause(3);

    errorDeg = 3;
    trajTime = 1.5;
    PV = [];
    
    writematrix(PV,'Data.csv');

    elapsed = 0;
    tic;
    for i = 2:4     
        
        if t == 3
            currentJoints = positions(i-1,:);
            goalJoints = positions(i,:);

        else
            currentJoints = robot.ik3001(positions(i-1,:));
            goalJoints = robot.ik3001(positions(i,:));
        end

        coeff1 = traj_planner.cubic_traj(0,trajTime,0,0,currentJoints(1),goalJoints(1));
        coeff2 = traj_planner.cubic_traj(0,trajTime,0,0,currentJoints(2),goalJoints(2));
        coeff3 = traj_planner.cubic_traj(0,trajTime,0,0,currentJoints(3),goalJoints(3));

        coeffMatrix = [coeff1 coeff2 coeff3];

        currTime = toc + elapsed;
        if t == 3 % task space
            PV = robot.run_trajectory(transpose(coeffMatrix), trajTime, true);
        end
        if t == 2 % joint space
            PV = robot.run_trajectory(transpose(coeffMatrix), trajTime, false);
        end
        
        if t == 1 % no interpolation
            goal = 0;
            n = 1;
            while(goal ~= 1)
                goal = robot.atGoalPos(2);
                robot.servo_jp(goalJoints);
                recorded = robot.measured_js(true,false);
                pos = robot.fk3001(transpose(recorded(1,:)));
                PV(n,1:3) = transpose(pos(1:3,4));
                n = n+1;
            end
        end
        writematrix(PV,'Data.csv','WriteMode','append');
        elapsed = currTime;

        pause(2);
    end  



    plotData = readmatrix('Data.csv');

    figure(1)
    plot3(plotData(:,1),plotData(:,2),plotData(:,3),'LineWidth', 2);
    hold on
    plot3(plotData(:,1),plotData(:,2),plotData(:,3),'LineWidth', 2);
    plot3(plotData(:,1),plotData(:,2),plotData(:,3),'LineWidth', 2);
    
    plotData = [];

end 
hold off
title('End Effector Path in mm');
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
legend('task space','joint space', 'no interpolation joint space');





robot.shutdown();