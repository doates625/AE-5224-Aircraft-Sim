classdef LKF < kalman.AbsKF
    %LKF Linear Kalman filter
    %   
    %   System model:
    %   x[n] = Fx*x[n-1] + Fu*u[n-1]
    %   z[n] = Hx*x[n]
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public)
        function obj = LKF(xh, Ex, Eu, Ez, Fx, Fu, Hx)
            %obj = KF(xh, Ex, Eu, Ez, Fx, Fu, Hx)
            %   Constuct Linear Kalman filter
            %   - xh = State estimate [n x 1]
            %   - Ex = State cov [n x n]
            %   - Eu = Input cov [m x m]
            %   - Ez = Output cov [p x p]
            %   - Fx = State matrix [n x n]
            %   - Fu = Input matrix [n x m]
            %   - Hx = Output matrix [p x n]
            obj@kalman.AbsKF(xh, Ex, Eu, Ez);
            obj.Fx = Fx;
            obj.Fu = Fu;
            obj.Hx = Hx;
        end
    end
    
    methods (Access = protected)
        function x = predict_x(obj, u)
            %x = PREDICT_X(obj, u)
            %   Predict state
            %   - u = Input vector [m x 1]
            %   - x = Predicted state [n x 1]
            x = obj.Fx * obj.xh + obj.Fu * u;
        end
        
        function z = predict_z(obj)
            %z = PREDICT_Z(obj, x)
            %   Predict output
            %   - z = Predicted output [p x 1]
            z = obj.Hx * obj.xh;
        end
    end
end