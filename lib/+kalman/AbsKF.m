classdef (Abstract) AbsKF < handle
    %ABSKF Superclass for Kalman Filters
    %   
    %   System dimensions:
    %   x = [n x 1]
    %   u = [m x 1]
    %   z = [p x 1]
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        x_est;  % State estimate [n x 1]
        cov_x;  % State cov [n x n]
        cov_u;  % Input cov [m x m]
        cov_z;  % Output covs [cell([p x p])]
        n_out;  % Number of outputs
    end
    
    properties (Access = protected)
        jac_xx; % State Jacobian [n x n]
        jac_xu; % Input Jacobian [n x m]
        jac_zx; % Output Jacobians [cell([p x n])]
        iden_n; % Identity matrix [n x n]
    end
    
    methods (Access = public)
        function obj = AbsKF(x_est, cov_x, cov_u, cov_z)
            %obj = ABSKF(x_est, cov_x, cov_u, cov_z)
            %   Construct Kalman Filter
            %   - x_est = State estimate [n x 1]
            %   - cov_x = State cov [n x n]
            %   - cov_u = Input cov [m x m]
            %   - cov_z = Output cov [p x p]
            %   For n > 1 outputs, make cov_z an [n x 1] cell.
            obj.x_est = x_est;
            obj.cov_x = cov_x;
            obj.cov_u = cov_u;
            obj.cov_z = obj.to_cell(cov_z);
            obj.n_out = length(obj.cov_z);
            obj.iden_n = eye(length(x_est));
        end
        
        function x_est = predict(obj, u)
            %x_est = PREDICT(obj, u)
            %   Prediction step
            %   - u = Input vector [m x 1]
            %   - x_est = Predicted state [n x 1]
            obj.x_est = obj.predict_x(u);
            obj.cov_x = ...
                obj.jac_xx * obj.cov_x * obj.jac_xx.' + ...
                obj.jac_xu * obj.cov_u * obj.jac_xu.';
            x_est = obj.x_est;
        end
        
        function x_est = correct(obj, z, i)
            %x_est = CORRECT(obj, z, id)
            %   Correction step
            %   - z = Output vector [p x 1]
            %   - x_est = Corrected state [n x 1]
            %   - id = Output index [1...n_out]
            z_exp = obj.predict_z(i);
            jac_zx_ = obj.jac_zx{i};
            K = (obj.cov_x * jac_zx_.') / ...
                (jac_zx_ * obj.cov_x * jac_zx_.' + obj.cov_z{i});
            obj.x_est = obj.x_est + K * (z - z_exp);
            obj.cov_x = (obj.iden_n - K * jac_zx_) * obj.cov_x;
            x_est = obj.x_est;
        end
    end
    
    methods (Access = protected, Abstract)
        x = predict_x(obj, u)
        %x = PREDICT_X(obj, u)
        %   Predict state
        %   - u = Input vector [m x 1]
        %   - x = Predicted state [n x 1]
        
        z = predict_z(obj, i)
        %z = PREDICT_Z(obj)
        %   Predict output
        %   - i = Output index [1...n_out]
        %   - z = Predicted output [p x 1]
    end
    
    methods (Access = protected, Static)
        function y = to_cell(x)
            %y = TO_CELL(x) Convert non-cells to cells
            if isa(x, 'cell')
                y = x;
            else
                y = {x};
            end
        end
    end
end