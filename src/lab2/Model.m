classdef Model < handle
    
    properties
        robot; 
    end
    
    methods
        
        function self = Model(robot)
            self.robot = robot; 
        end
       
        function FK = FKdraw(self, jointVal)

            T = self.robot.tMatricies(jointVal);     % Transformation Matrix
            p = self.robot.posVectorCoord(jointVal); % Position vector
            disp(p);
            plot3(T(1,:), T(2, :), T(3, :), '-o','LineWidth', 2, ...
                'MarkerSize',6,'MarkerFaceColor',[0.5,1,0.5]);
            
            hold on
            
            % pChange is the amount we are moving the vector from where it starts.
            pChange = [20 0 0;
                       0 20 0;
                       0 0 20];
                       
            % Plotting all vectors
            frame0x = [0 0 0; 20 0 0;]; %0 0 0; 0 0 55; 20 0 55];
            frame0y = [0 0 0; 0 20 0;]; %0 0 0; 0 0 55; 0 20 55];
            frame0z = [0 0 0; 0 0 20;]; %0 0 0; 0 0 55; 0 0 75];
            
            transformMatrices = self.robot.baseToTransform(jointVal);
            
            pTransf1 = transformMatrices(1:3, 1:3)^(-1);
            pTransf2 = transformMatrices(1:3, 5:7)^(-1);
            pTransf3 = transformMatrices(1:3, 9:11)^(-1);
            pTransf4 = transformMatrices(1:3, 13:15)^(-1);
            
            pChange1 = pTransf1*pChange;
            pChange2 = pTransf2*pChange;
            pChange3 = pTransf3*pChange;
            pChange4 = pTransf4*pChange;
            
            %Base Frame
            plot3(frame0x(:, 1), frame0x(:, 2), frame0x(:, 3), '-o', ...
                  frame0y(:,1), frame0y(:,2), frame0y(:,3), '-o', ...
                  frame0z(:,1), frame0z(:,2), frame0z(:,3), '-o');
                  
            % Frame 1 vectors
            plot3([p(1,1), p(1,1) + pChange1(1,1)], [p(1,2), p(1,2) + pChange1(1,2)], [p(1,3), p(1,3)+pChange1(1,3)], '-o', ...
                  [p(1,1), p(1,1) + pChange1(2,1)], [p(1,2), p(1,2) + pChange1(2,2)], [p(1,3), p(1,3)+pChange1(2,3)], '-o', ...
                  [p(1,1), p(1,1) + pChange1(3,1)], [p(1,2), p(1,2) + pChange1(3,2)], [p(1,3), p(1,3)+pChange1(3,3)], '-o');
            
            % Frame 2
            plot3([p(2,1), p(2,1) + pChange2(1,1)], [p(2,2), p(2,2) + pChange2(1,2)], [p(2,3), p(2,3)+pChange2(1,3)], '-o', ...
                  [p(2,1), p(2,1) + pChange2(2,1)], [p(2,2), p(2,2) + pChange2(2,2)], [p(2,3), p(2,3)+pChange2(2,3)], '-o', ...
                  [p(2,1), p(2,1) + pChange2(3,1)], [p(2,2), p(2,2) + pChange2(3,2)], [p(2,3), p(2,3)+pChange2(3,3)], '-o');
            
            % Frame 3
            plot3([p(3,1), p(3,1) + pChange3(1,1)], [p(3,2), p(3,2) + pChange3(1,2)], [p(3,3), p(3,3)+pChange3(1,3)], '-o', ...
                  [p(3,1), p(3,1) + pChange3(2,1)], [p(3,2), p(3,2) + pChange3(2,2)], [p(3,3), p(3,3)+pChange3(2,3)], '-o', ...
                  [p(3,1), p(3,1) + pChange3(3,1)], [p(3,2), p(3,2) + pChange3(3,2)], [p(3,3), p(3,3)+pChange3(3,3)], '-o');
            
            % Frame 4
            plot3([p(4,1), p(4,1) + pChange4(1,1)], [p(4,2), p(4,2) + pChange4(1,2)], [p(4,3), p(4,3)+pChange4(1,3)], '-o', ...
                  [p(4,1), p(4,1) + pChange4(2,1)], [p(4,2), p(4,2) + pChange4(2,2)], [p(4,3), p(4,3)+pChange4(2,3)], '-o', ...
                  [p(4,1), p(4,1) + pChange4(3,1)], [p(4,2), p(4,2) + pChange4(3,2)], [p(4,3), p(4,3)+pChange4(3,3)], '-o');
          
            grid on; 
            axis([-50,200,-200,250,0,295]);

            title('Stem Plot')
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Z Axis');
            
            h = rotate3d;
            h.Enable = 'on';
            hold off

        end
        
        % Plots the workspace of the robotic arm.
        function plotWorkspace(self, jointVal)
            T = self.robot.tMatricies(jointVal);      % Transformation Matrix
            
            % We don't want points behind, or below the arm.
            if T(1, 5) > -10 && T(3, 5) > 0
                plot3(T(1,5), T(2, 5), T(3, 5), '-o','LineWidth', 2, ...
                    'MarkerSize',6,'MarkerFaceColor',[0.5,1,0.5]);
            end
            
            hold on
            
            grid on; 
            axis([-50,200,-200,250,0,295]);

            title('Stem Plot')
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Z Axis');
            
            h = rotate3d;
            h.Enable = 'on';
            hold off
        end
            
        function mypostcallback(obj,evd)
            %disp('A rotation is about to occur.');
            ax_properties = get(gca);
            assignin('base','pov',ax_properties.View);
        end
   end
end
                