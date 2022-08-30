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


startPoints = [0, 0, 0]; %home values
robot.servo_jp(startPoints); % send to home position
pause(1);

interpolateTime = 2000; % time of movement in ms
currentTime = 0;        % elapsed time
newTime = 0;

setPoints = [60, 60, 60]; % movements of whole arm




for t = 1:10
    
    posMatrix = zeros(i, 3);
    timeMatrix = zeros(i, 1);
    deltaT = zeros(i, 1);
    
    robot.servo_jp(setpoints);
    

    robot.interpolate_jp(startPoints, interpolateTime);

    goalPos = robot.goal_js();
    currentPos = zeros(2, 3);
    errorDeg = 1; % prevents undershooting
    
    tic % start timer (when does this start?)
    i = 1; %counter

    while robot.atGoalPos == false 

        currentPos = robot.measured_js(true, false); % assign current position


        posMatrix(i, :) = currentPos(1, :); %index position

        timeMatrix(i, 1) = toc; %index current time
        deltaT(i, 1) = toc - newTime; %update time
        newTime = deltaT(i, 1);

        i = i + 1;
    end

    % Making the datamatrix on the csv
    dataMatrix(:,1) = timeMatrix;
    dataMatrix(:,2:4) = posMatrix;
    writematrix(dataMatrix, 'posData.csv'); 

    for i = 1:3
        figure(i)
        % plot(dataMatrix(:, 1), dataMatrix(:, i + 1), 'LineWidth', 2);

        %Need to figure out how to read and manipulate a csv file.
        M = readmatrix('posData.csv');
        plot(M(:, 1), M(:, i + 1), 'LineWidth', 2);

        axis([0 (interpolateTime / 1000) -5 45]);

        iStr = int2str(i);

        title("Motor " + iStr + " Positions vs Time");
        xlabel("Time(s)");
        ylabel("Motor Positions(degrees)");
        legend("Motor " + iStr);
    end
end
robot.shutdown()