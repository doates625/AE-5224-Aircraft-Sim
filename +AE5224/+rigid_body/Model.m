classdef (Abstract) Model < handle
    %BODY Class for 3D rigid body models
    %   
    %   State vector x:
    %   - p_e = Position [Earth, m]
    %   - q_e = Attitude [Earth, quaternion]
    %   - v_e = Linear velocity [Earth, m]
    %   - w_b = Angular velocity [Body, rad/s]
    %   
    %   Input vector u:
    %   - F_b = Net forces Body-frame [N]
    %   - M_b = Net moments Body-frame [N*m]
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        m;      % Mass [kg]
        I_b;    % Body-fixed inertia [kg*m^2]
    end
    
    methods (Access = public, Static)
        function x = pack_x(p_e, q_e, v_e, w_b)
            %x = PACK_X(p_e, q_e, v_e, w_b)
            %   Combines states into single vector
            %   
            %   Inputs:
            %   - p_e = Position [Earth, m]
            %   - q_e = Attitude [Earth, quaternion]
            %   - v_e = Linear velocity [Earth, m]
            %   - w_b = Angular velocity [Body, rad/s]
            %   
            %   Outputs:
            %   - x = State vector
            x = [p_e; q_e; v_e; w_b];
        end
        
        function [p_e, q_e, v_e, w_b] = unpack_x(x)
            %[p_e, q_e, v_e, w_b] = UNPACK_X(x)
            %   Extracts states from single vector
            %   
            %   Inputs:
            %   - x = State vector
            %   
            %   Outputs:
            %   - p_e = Position [Earth, m]
            %   - q_e = Attitude [Earth, quaternion]
            %   - v_e = Linear velocity [Earth, m]
            %   - w_b = Angular velocity [Body, rad/s] 
            p_e = x(1:3);
            q_e = x(4:7);
            v_e = x(8:10);
            w_b = x(11:13);
        end
    end
    
    methods (Access = public)
        function obj = Model(m, I_b)
            %obj = MODEL(m, I_b)
            %   Construct rigid body model
            %   
            %   Inputs:
            %   - m = Mass [kg]
            %   - I_b = Body-fixed inertia [kg*m^2]
            obj.m = m;
            obj.I_b = I_b;
        end
        
        function [F_b, M_b] = forces(~, ~, u)
            %[F_b, M_b] = FORCES(obj, x, u)
            %   Compute body-frame net forces and moments
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   - u = Input vector [F_b; M_b]
            %   
            %   Outputs:
            %   - F_b = Net force vector [N]
            %   - M_b = Net moment vector [N*m]
            F_b = u(1:3);
            M_b = u(4:6);
        end
        
        function x_dot = dynamics(obj, x, u)
            %x_dot = DYNAMICS(obj, x, u)
            %   Compute state derivative
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   - u = Input vector [F_b; M_b]
            %   
            %   Outputs:
            %   - x_dot = State derivative
            [F_b, M_b] = obj.forces(x, u);
            x_dot = obj.force_dynamics(x, F_b, M_b);
        end
    end
    
    methods (Access = protected)
        function x_dot = force_dynamics(obj, x, F_b, M_b)
            %x_dot = FORCE_DYNAMICS(obj, x, F_b, M_b)
            %   Compute state derivative from body-frame net forces and moments
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   - F_b = Net force vector [N]
            %   - M_b = Net moment vector [N*m]
            %   
            %   Outputs:
            %   - x_dot = State derivative
            
            % Imports
            import('quat.Quat');
            
            % Unpack state
            [~, q_e, v_e, w_b] = obj.unpack_x(x);
            
            % Position and attitude
            p_e_dot = v_e;
            q_e_dot = Quat(q_e) * Quat([0; 0.5*w_b]);
            q_e_dot = q_e_dot.vector();
            
            % Linear velocity
            q_e = Quat(q_e);
            F_e = q_e.rotate(F_b);
            v_e_dot = F_e / obj.m;
            
            % Angular velocity
            L_b = obj.I_b * w_b;
            L_b_dot = M_b - cross(w_b, L_b);
            w_b_dot = obj.I_b \ L_b_dot;
            
            % Pack states
            x_dot = obj.pack_x(p_e_dot, q_e_dot, v_e_dot, w_b_dot);
        end
    end
end