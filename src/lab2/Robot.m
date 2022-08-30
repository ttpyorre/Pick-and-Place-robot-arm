classdef Robot < handle

    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
        
        SERV_ID = 1848
        SERVER_ID_READ_POS = 1910; % ID of position packet
        SERVER_ID_READ_VEL = 1822; % ID of velocity packet
        
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
            L0 = 55;
            L1 = 40;
            L2 = 100;
            L3 = 100;
            
            DHtable = [jointConfig(1, 1), L1, 0, -(90);
                       jointConfig(2, 1)-(90), 0, L2, 0;
                       jointConfig(3, 1)+(90), 0, L3, 0];
            
            Tbase = [1,0,0,0;  % identity 
                     0,1,0,0;
                     0,0,1,L0;
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
            L0 = 55;
            L1 = 40;
            L2 = 100;
            L3 = 100;
            
            DHtable = [jointConfig(1, 1), L1, 0, -(90);
                       jointConfig(2, 1)-(90), 0, L2, 0;
                       jointConfig(3, 1)+(90), 0, L3, 0];
                   
            Tbase = [1,0,0,0;  % identity 
                     0,1,0,0;
                     0,0,1,L0;
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
            
            L0 = 55;
            L1 = 40;
            L2 = 100;
            L3 = 100;
            
            DHtable = [jointConfig(1, 1), L1, 0, -(90);
                       jointConfig(2, 1)-(90), 0, L2, 0;
                       jointConfig(3, 1)+(90), 0, L3, 0];
                   
            Tbase = [1,0,0,0;  % identity 
                     0,1,0,0;
                     0,0,1,L0;
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
            L1 = 40;
            L2 = 100;
            L3 = 100;
            
            DHtable = [jointConfig(1, 1), L1, 0, -(90);
                       jointConfig(2, 1)-(90), 0, L2, 0;
                       jointConfig(3, 1)+(90), 0, L3, 0];
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
%%        
        
    end
end
