classdef Traj_Planner < handle
    
    properties
        % lol
    end
    
    methods
        function self = Traj_Planner()
            % Nothing yet.
        end
        
        % Input: Inital and final values of time, velocity, and position.
        % Output: Coefficients of the cubic trajectory polynomial.
        function coefficients = cubic_traj(self,t0,tf,v0,vf,p0,pf) 
            
            M = [1, t0, t0^2, t0^3;
                 0, 1,  2*t0, 3*t0^2;
                 1, tf, tf^2, tf^3;
                 0, 1,  2*tf, 3*tf^2];
 
             
            Q = [p0; v0; pf; vf];
            
            
            coefficients = M^(-1)*Q;
            return
        end
        
        % Does the same as previos, but includes acceleration, and outputs
        % more coefficients.
        function coefficients = quintic_traj(self,t0,tf,v0,vf,p0,pf,a0,af) 
            
            M = [1, t0, t0^2, t0^3, t0^4, t0^5;
                 0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
                 0, 0, 2, 6*t0, 12*t0^2,20*t0^3;  
                 1, tf, tf^2, tf^3,tf^4,tf^5;
                 0, 1,  2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
                 0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
             
            Q = [p0; v0; a0; pf; vf; af];
            
            coefficients = M^(-1)*Q;
            return
        end
        
   end
end
                