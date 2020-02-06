classdef Sim < AE5224.rigid_body.Sim
    %SIM Class for quadrotor simulations
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public)
        function update(obj, w1, w2, w3, w4)
            %UPDATE(obj, wp)
            %   Run one simulation loop
            %   
            %   Inputs:
            %   - w1 = Prop 1 rate [rpm]
            %   - w2 = Prop 2 rate [rpm]
            %   - w3 = Prop 3 rate [rpm]
            %   - w4 = Prop 4 rate [rpm]
            
            % Imports
            import('AE5224.get_g');
            import('quat.Quat');
            
            % Convert to forces and moments
            wp2 = [w1; w2; w3; w4].^2;
            F_wp = obj.body.k_F * wp2;
            M_wp = obj.body.k_M * wp2;
            
            % Net force body-frame
            F_mg_e = [0; 0; obj.body.m * get_g()];
            F_mg_b = Quat(obj.q_e).inv().rotate(F_mg_e);
            F_wp_b = [0; 0; -sum(F_wp)];
            F_b = F_mg_b + F_wp_b;
            
            % Net moment body-frame
            M_b = zeros(3, 1);
            M_b(1) = obj.body.L * (F_wp(2) - F_wp(4));
            M_b(2) = obj.body.L * (F_wp(3) - F_wp(1));
            M_b(3) = [+1, -1, +1, -1] * M_wp;
            
            % Update rigid body
            update@AE5224.rigid_body.Sim(obj, F_b, M_b);
        end
    end
end