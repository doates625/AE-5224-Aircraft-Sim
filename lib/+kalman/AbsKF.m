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
        xh; % State estimate [n x 1]
        Ex; % State cov [n x n]
        Eu; % Input cov [m x m]
        Ez; % Output cov [p x p]
    end
    
    properties (Access = protected)
        Fx; % State Jacobian [n x n]
        Fu; % Input Jacobian [n x m]
        Hx; % Output Jacobian [p x n]
        In; % Identity matrix [n x n]
    end
    
    methods (Access = public)
        function obj = AbsKF(xh, Ex, Eu, Ez)
            %obj = ABSKF(x, Ex, Eu, Ez)
            %   Construct Kalman Filter
            %   - xh = State estimate [n x 1]
            %   - Ex = State cov [n x n]
            %   - Eu = Input cov [m x m]
            %   - Ez = Output cov [p x p]
            obj.xh = xh;
            obj.Ex = Ex;
            obj.Eu = Eu;
            obj.Ez = Ez;
            obj.In = eye(length(xh));
        end
        
        function xh = predict(obj, u)
            %xh = PREDICT(obj, u)
            %   Prediction step
            %   - u = Input vector [m x 1]
            %   - xh = Predicted state [n x 1]
            obj.xh = obj.predict_x(u);
            obj.Ex = obj.Fx * obj.Ex * obj.Fx.' + obj.Fu * obj.Eu * obj.Fu.';
            xh = obj.xh;
        end
        
        function xh = correct(obj, z)
            %xh = CORRECT(obj, z)
            %   Correction step
            %   - z = Output vector [p x 1]
            %   - xh = Corrected state [n x 1]
            zh = obj.predict_z();
            K = (obj.Ex * obj.Hx.') / (obj.Hx * obj.Ex * obj.Hx.' + obj.Ez);
            obj.xh = obj.xh + K * (z - zh);
            obj.Ex = (obj.In - K * obj.Hx) * obj.Ex;
            xh = obj.xh;
        end
    end
    
    methods (Access = protected, Abstract)
        x = predict_x(obj, u)
        %x = PREDICT_X(obj, u)
        %   Predict state
        %   - u = Input vector [m x 1]
        %   - x = Predicted state [n x 1]
        
        z = predict_z(obj)
        %z = PREDICT_Z(obj)
        %   Predict output
        %   - z = Predicted output [p x 1]
    end
end