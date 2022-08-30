classdef Robot < handle

    properties
        myHIDSimplePacketComs;
        pol;
        model;
        
        GRIPPER_ID = 1962;
        SERV_ID = 1848;
        SERVER_ID_READ_POS = 1910; % ID of position packet
        SERVER_ID_READ_VEL = 1822; % ID of velocity packet
        
        L0 = 55;
        L1 = 40;
        L2 = 100;
        L3 = 100;
        
        radToDeg = 180/pi;
        goal = [0, 0, 0];
        
    end
    
    methods
        
        %The is a shutdown function to clear the HID hardware connection
        function shutdown(self)
	    %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
            self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
            self.model = Model(self);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
            com = zeros(15, 1, 'single');
            try
                ds = javaArray('java.lang.Double',length(values));
                for i = 1:length(values)
                    ds(i) = java.lang.Double(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                ret = self.myHIDSimplePacketComs.readFloats(intid) ;
                for i = 1:length(com)
                   com(i) = ret(i).floatValue();
                end
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        function com = read(self, idOfCommand)
            com = zeros(15, 1, 'single');
            try

                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                for i = 1:length(com)
                   com(i) = ret(i).floatValue();
                end
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        function  write(self, idOfCommand, values)
            try
                ds = javaArray('java.lang.Double',length(values));
                for i = 1:length(values)
                    ds(i) = java.lang.Double(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                self.myHIDSimplePacketComs.writeFloats(intid, ds, self.pol);

            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
            
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(0);
        end
        
        
        %% Lab 1 Functions
        % Sends joint values directly to Actuators, bypassing
        % interpolation.
        function servo_jp(self, inputArray)
            self.goal = inputArray;
            packet = zeros(15, 1, 'single');
            
            packet(3) = inputArray(1);
            packet(4) = inputArray(2);
            packet(5) = inputArray(3);
            
            self.write(1848, packet);
        end
        
        % Sets position of arm joints with a certain interpolation time (ms)
        % takes a 1x3 array to each each joint, and the time in milliseconds
        function interpolate_jp(self, posArray, timeMS)
           
            self.goal = posArray;
            packet = zeros(15,1,'single');
            
            packet(1) = timeMS; %interpolation time in ms
            packet(2) = 0;      %linear interpolation
            
            packet(3) = posArray(1);  % first link to position
            packet(4) = posArray(2);  % second link to position
            packet(5) = posArray(3);  % third link to position
            
            self.write(self.SERV_ID, packet);
        end
        
        % Gives the values of position and velocity depending on which we want.
        function measurement = measured_js(self, GETPOS, GETVEL)
            % Reads from the position data.
            pos = self.read(self.SERVER_ID_READ_POS);
            % Reads from the velocity data.
            vel = self.read(self.SERVER_ID_READ_VEL);
            
            % Where the data will be stored.
            measurement = zeros(2,3);
            
            try
               measurement(1, :) = [pos(3), pos(5), pos(7)];
               measurement(2, :) = [vel(3), vel(6), vel(9)];
               
               if GETPOS == false
                   measurement(1, :) = 0;
               end
               
               if GETVEL == false
                   measurement(2, :) = 0;
               end
               
            catch
                getReport(exception)
                disp('measured_js went wrong!!!');
            end
        end
        
        % Return a 1x3 array that contains current joint set point positions in degrees.
        function jointPositions = setpoint_js(self)
            positionsInDegrees = self.read(1910);
            jointPositions = [positionsInDegrees(2), positionsInDegrees(4), positionsInDegrees(6)];
            return
        end
        
        % Gets the goal where we want to arrive at.
        function giveGoal = goal_js(self)
            giveGoal = self.goal;
            return
        end 
        
        
        % takes a margin of error in degrees and returns true if the current position
        % is within that error 
        %returns false otherwise
        function goal = atGoalPos(self,errorDeg)
            
            goalPos = self.goal_js();
            currentPos = self.measured_js(true,false);
            goal = false;
            
            if abs(goalPos(1) - currentPos(1,1)) < errorDeg && ...
               abs(goalPos(2) - currentPos(1,2)) < errorDeg && ...
               abs(goalPos(3) - currentPos(1,3)) < errorDeg
                goal = true;
            end
            return
        end
        
        %% LAB 2 Functions
        
        % Using first row of DH table to convert into a transformation matrix.
        function t = dh2mat(self, jointVar) % jointVar is a 1x4 matrix representing theta, d, a, and alpha
            theta = jointVar(1,1);
            d = jointVar(1,2);
            a = jointVar(1,3);
            alpha = jointVar(1,4);
            t = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
                 sind(theta) cosd(theta)*cosd(alpha)  -cosd(theta)*sind(alpha) a*sind(theta);
                 0,sind(alpha),cosd(alpha),d;
                 0,0,0,1];
             return
        end
        
        % What do
        %input : nx4 DHtable
        %output: 4x4 transformation matrix form base frame to the final
        %frame
        function t = dh2fk(self, DHtable) % nx4 matrix representing theta d a alpha respectively
            t = 1;
            tableSize = size(DHtable);
            rowSize = tableSize(1, 1);
            
            for i = 1:rowSize
                link = DHtable(i, :);
                t = t * self.dh2mat(link);
            end
            return
        end
        
        % take in 3 angles, theta_1, theta_2, theta_3
        % output transformation matricies t01, t02, t03, t04
        function T = tMatricies(self,jointConfig)
        
            
            DHtable = [jointConfig(1, 1), self.L1, 0, -(90);
                       jointConfig(2, 1)-(90), 0, self.L2, 0;
                       jointConfig(3, 1)+(90), 0, self.L3, 0];
            
            Tbase = [1,0,0,0;  % identity 
                     0,1,0,0;
                     0,0,1,self.L0;
                     0,0,0,1];
                 
            % Making Reference frames:     
            T12 = self.dh2mat([DHtable(1,1),DHtable(1,2), ...
                              DHtable(1,3),DHtable(1,4)]);
                        
            T23 = self.dh2mat([DHtable(2,1),DHtable(2,2), ...
                              DHtable(2,3),DHtable(2,4)]);
                        
            T34 = self.dh2mat([DHtable(3,1),DHtable(3,2), ...
                              DHtable(3,3),DHtable(3,4)]);
            
            % From frame 0 to X
            T01 = Tbase;
            T02 = T01*T12;
            T03 = T02*T23;
            T04 = T03*T34;
            
            % Putting the frame matrices into a vector.
            t = [T01 T02 T03 T04];
            % Giant 4x16 Matrix.
            
            % T1 has the positions of the transformation matrices.
            T1 = [0 t(1,4) t(1,8) t(1,12) t(1,16)]; 
            T2 = [0 t(2,4) t(2,8) t(2,12) t(2,16)]; 
            T3 = [0 t(3,4) t(3,8) t(3,12) t(3,16)]; 
            T = [T1; T2; T3];
            % T = [0 d11 d12 d13; 0 d21 d22 d23; 0 d31 d32 d33]
        end
        
        % Write a comment pls, what does this do ??
        function p = posVectorCoord(self,jointConfig)
      
            DHtable = [jointConfig(1, 1), self.L1, 0, -(90);
                       jointConfig(2, 1)-(90), 0, self.L2, 0;
                       jointConfig(3, 1)+(90), 0, self.L3, 0];
                   
            Tbase = [1,0,0,0;  % identity 
                     0,1,0,0;
                     0,0,1,self.L0;
                     0,0,0,1];   
            
            % Make Transformation Matrix frames
            T12 = self.dh2mat([DHtable(1,1),DHtable(1,2), ...
                              DHtable(1,3),DHtable(1,4)]);
                        
            T23 = self.dh2mat([DHtable(2,1),DHtable(2,2), ...
                              DHtable(2,3),DHtable(2,4)]);
                        
            T34 = self.dh2mat([DHtable(3,1),DHtable(3,2), ...
                              DHtable(3,3),DHtable(3,4)]);
            
            % Make the Transformation matrices into a big matrix.
            T01 = Tbase;
            T02 = T01*T12;
            T03 = T02*T23;
            T04 = T03*T34;
            
            % Make a coordinate matrix.
            p = [T01(1,4),T01(2,4),T01(3,4); %xyz joint 1
                 T02(1,4),T02(2,4),T02(3,4); %xyz joint 2
                 T03(1,4),T03(2,4),T03(3,4); %etc...
                 T04(1,4),T04(2,4),T04(3,4)];
        end 
        
        %input frame number f
        %output 4x4 transformation matrix
        function p = baseToTransform(self, jointConfig)
            

            DHtable = [jointConfig(1, 1), self.L1, 0, -(90);
                       jointConfig(2, 1)-(90), 0, self.L2, 0;
                       jointConfig(3, 1)+(90), 0, self.L3, 0];
                   
            Tbase = [1,0,0,0;  % identity 
                     0,1,0,0;
                     0,0,1,self.L0;
                     0,0,0,1];   
            
            % Make Transformation Matrix frames
            T12 = self.dh2mat([DHtable(1,1),DHtable(1,2), ...
                              DHtable(1,3),DHtable(1,4)]);
                        
            T23 = self.dh2mat([DHtable(2,1),DHtable(2,2), ...
                              DHtable(2,3),DHtable(2,4)]);
                        
            T34 = self.dh2mat([DHtable(3,1),DHtable(3,2), ...
                              DHtable(3,3),DHtable(3,4)]);
            
            % Make the Transformation matrices into a big matrix.
            T01 = Tbase;
            T02 = T01*T12;
            T03 = T02*T23;
            T04 = T03*T34;
            
            p = [T01 T02 T03 T04];
            
        end
        % Solves Forward Kinematics for given joint space.
        function t = fk3001(self, jointConfig) % 3x1 array of angles 
            J1 = jointConfig(1);
            J2 = jointConfig(2);
            J3 = jointConfig(3);
            
            t = [ cos((pi*J1)/180)*cos((pi*(J2 - 90))/180)*cos((pi*(J3 + 90))/180) - cos((pi*J1)/180)*sin((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180), - cos((pi*J1)/180)*cos((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180) - cos((pi*J1)/180)*cos((pi*(J3 + 90))/180)*sin((pi*(J2 - 90))/180), -sin((pi*J1)/180), 100*cos((pi*J1)/180)*cos((pi*(J2 - 90))/180) + 100*cos((pi*J1)/180)*cos((pi*(J2 - 90))/180)*cos((pi*(J3 + 90))/180) - 100*cos((pi*J1)/180)*sin((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180);
                  sin((pi*J1)/180)*cos((pi*(J2 - 90))/180)*cos((pi*(J3 + 90))/180) - sin((pi*J1)/180)*sin((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180), - sin((pi*J1)/180)*cos((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180) - sin((pi*J1)/180)*cos((pi*(J3 + 90))/180)*sin((pi*(J2 - 90))/180),  cos((pi*J1)/180), 100*sin((pi*J1)/180)*cos((pi*(J2 - 90))/180) + 100*sin((pi*J1)/180)*cos((pi*(J2 - 90))/180)*cos((pi*(J3 + 90))/180) - 100*sin((pi*J1)/180)*sin((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180);
                  - cos((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180) - cos((pi*(J3 + 90))/180)*sin((pi*(J2 - 90))/180),                                     sin((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180) - cos((pi*(J2 - 90))/180)*cos((pi*(J3 + 90))/180),                 0,                                               95 - 100*cos((pi*(J2 - 90))/180)*sin((pi*(J3 + 90))/180) - 100*cos((pi*(J3 + 90))/180)*sin((pi*(J2 - 90))/180) - 100*sin((pi*(J2 - 90))/180);
                                                                                                                                   0,                                                                                                                                     0,                 0,                                                                                                                                                                                          1];
            
            return
        end
        
        % Assigns the DHtable.
        function d = assignTable(self, jointConfig)

            DHtable = [jointConfig(1, 1), self.L1, 0, -(90);
                       jointConfig(2, 1)-(90), 0, self.L2, 0;
                       jointConfig(3, 1)+(90), 0, self.L3, 0];
            d = DHtable;
        end
        
        % What do
        function t = measured_cp(self)
            positionArray = transpose(self.measured_js(1,0));
            t = self.fk3001(positionArray);
        end
        
        % What do
        function t = setpoint_cp(self)
            setPointPos = transpose(self.setpoint_js());
            t = self.fk3001(setPointPos);
        end
        
        % What do
        function t = goal_cp(self)
            goalMat = transpose(self.goal_js());
            t = self.fk3001(goalMat);
        end  
        
        %% Lab 3 Fucntions -- Inverse Kinematics, Joint and Task space Trajectory

        % Takes a  joint value and upper and lower limit values and returns the first
        % value with in the given range
        % returns 365 if there is no value within range 
        function j = checkLimits(self, jointArray,lowerlimit,upperlimit)
            j = 365; %returns if no valid value is found
            if jointArray > lowerlimit && jointArray < upperlimit %checks if in range
                j = jointArray;
            else
                disp("Inverse Kinematics out of Bounds.");
                self.shutdown();
            end 

            return 
        end 

        % Takes a 1x3 array of end effector position values and returns an array
        % of joint values in degrees to get to that position 
        % (joint1, joint2,joint3)
        function  t = ik3001(self,eePos)
            x = eePos(1);
            y = eePos(2);
            z = eePos(3);
            xEndEffector = sqrt(x^2+y^2);
            zEndEffector = z - (self.L0 + self.L1); % z of triangle at end effector

            % Theta of Joint 1
            possibleJ1 = atan2(y, x);
            theta1 = self.checkLimits(possibleJ1,-100,100)*self.radToDeg; %limits found through testing

            % Theta of Joint 2
            cosBetaT2 = (self.L2^2+xEndEffector^2+zEndEffector^2-(self.L3)^2)/(2*self.L2*(sqrt(xEndEffector^2+zEndEffector^2)));
            sinBetaT2 = sqrt(1-cosBetaT2^2); % Beta

            cosAlphaT2 = zEndEffector/(sqrt(xEndEffector^2+zEndEffector^2));
            sinAlphaT2 = sqrt(1-cosAlphaT2^2); % Alpha

            betaT2 = atan2(sinBetaT2, cosBetaT2);
            alphaT2 = atan2(sinAlphaT2, cosAlphaT2);
            % This is the angle we get for Theta of Joint 2.
            possibleTheta2 = alphaT2 - betaT2;

            theta2 = possibleTheta2*self.radToDeg;

            % Theta of Joint 3
            cosTheta3 = (self.L2^2 + self.L3^2 - (xEndEffector^2+zEndEffector^2))/(2*self.L2*self.L3);
            sinTheta3 = sqrt(1-cosTheta3^2);

            possibleJ3 = atan2(sinTheta3, cosTheta3);
            theta3 = -(self.checkLimits(possibleJ3, -104, 78) - pi/2)*self.radToDeg;

            % Checking for errors
%             theta1 > 110 || theta1 < -95) || ...
%                (theta2 > 85  || theta2 < -35) || ...
%                (theta3 > 60  || theta3 < -80)
            if (theta1 > 110 || theta1 < -95) || ...
               (theta2 > 85  || theta2 < -35) || ...
               (theta3 > 60  || theta3 < -80)
                disp(theta1); disp(theta2); disp(theta3);
                t = [0, 0, 0];
                disp('Inverse Kinematics failed.');
                return
            end 
            t = [theta1, theta2, theta3];
            return
        end
        
        % Passing in 3X4 matrix of coefficiences, time we want the program
        % to take, and a boolean of it being in joint space or not.
        function PV = run_trajectory(self, trajCoeff, time, inJointSpace)
            tic;
            
            % Current time in the trajectory
            timeOff = toc;
            
            
            %time - timeoff is the remaining of the trajectory
            while time - timeOff > 0
                
                % Calculating the positions where the arm should go, at a given time.
                posJoint1orX = self.calculateTrajPos(trajCoeff(1, :), timeOff);
                posJoint2orY = self.calculateTrajPos(trajCoeff(2, :), timeOff);
                posJoint3orZ = self.calculateTrajPos(trajCoeff(3, :), timeOff);
                
                input = [posJoint1orX posJoint2orY posJoint3orZ];
                if inJointSpace

                    input = self.ik3001(input);
 

                end

                self.servo_jp(input);

                pause(0.06);
                timeOff = toc;
            end

            return
        end
        
        %%% 3 Functions that can calculate any trajCoeff for position, velocity, and acceleration. %%%
        function p = calculateTrajPos(self,trajCoeff,time)
            p = 0;
            % All these functions can be represented by a series.
            for i = 1:length(trajCoeff)
                p = p + trajCoeff(i)*time^(i-1);
            end
            return
        end
        
        function v = calculateTrajVel(self,trajCoeff,time)
            v = 0;
            for i = 2:length(trajCoeff)
                v = v + (i-1)*trajCoeff(i)*time^(i-2);
            end
            return
        end
        
        function a = calculateTrajAcc(self,trajCoeff,time)
            a = 0;
            for i = 3:length(trajCoeff)
                a = a + (i-1)*(i-2)*trajCoeff(i)*time^(i-3); 
            end
            return
        end
        
        %% Lab 4 Functions
        %pog
        function pog(self)
            disp("gamer");
        end
        
        %takes a 1x3 matrix of joint angles and returns the 6x3 Jacobian
        %matrix
        function J = jacob3001(self, jointAngles)
           J1 = jointAngles(1);
           J2 = jointAngles(2); 
           J3 = jointAngles(3); 
           
           % Calculate Upper Jacobian
           Jp = [-100*sind((J1))*(cosd(((J2 + J3))) + sind((J2))), ...
                 -100*cosd((J1))*(sind(((J2 + J3))) - cosd((J2))), ...
                 -100*sind(((J2 + J3)))*cosd((J1)); % First row
                  100*cosd((J1))*(cosd(((J2 + J3))) + sind((J2))), ...
                 -100*sind((J1))*(sind(((J2 + J3))) - cosd((J2))), ...
                 -100*sind(((J2 + J3)))*sind((J1)); % Second row
                  0, -100*cosd(((J2 + J3))) - 100*sind((J2)), -100*cosd(((J2 + J3)))]; % Third Row
           %disp(det(Jp));
           self.checkSingularity(Jp);
                                                 
           % Calculate Lower Jacobian
           Jo = [0,-sind(J1),-sind(J1);
                 0, cosd(J1), cosd(J1);
                 1, 0,        0];
           
           J = [Jp;Jo];
           return
        end
        
        % Checks if the Jacobian goes close to a singularity.
        function checkSingularity(self, upperJacobian)
            determinant = det(upperJacobian);
            %disp(determinant);
            if  abs(determinant) < 10000
                self.shutdown();
                msg = "Too close to a singularity.";
                error(msg);
            end
        end
        
        % Takes two 1x3 matricies of joint angles and velocities and returns
        % a 6x1 vector of task space linear velocity and angular velocity
        function dp = fdk3001(self, jointAngles, jointVelocities)
            J = self.jacob3001(jointAngles);
            dq = jointVelocities';
            dp = J*dq;
            return
        end
        
        %% Functions from the final project.
        function gamer(self)
            self.pog();
        end
        
    end
end
