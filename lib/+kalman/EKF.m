classdef EKF < kalman.AbsKF
    %EKF Extended Kalman filter
    %   
    %   System model:
    %   - x[n] = f(x[n-1], u[n-1])
    %   - z[n] = h(x[n])
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        f;      % State func [Rn, Rm -> Rn]
        h;      % Output func [Rn -> Rp]
        Fx_;    % State Jacobian func [Rn, Rm -> Rnn]
        Fu_;    % Input Jacobian func [Rn, Rm -> Rnm]
        Hx_;    % Output Jacobian func [Rn -> Rpn]
    end
    
    methods (Access = public)
        function obj = EKF(xh, Ex, Eu, Ez, f, h, Fx, Fu, Hx)
            %obj = EKF(xh, Ex, Eu, Ez, f, h)
            %   Construct Extended Kalman filter
            %   - xh = State estimate [n x 1]
            %   - Ex = State cov [n x n]
            %   - Eu = Input cov [m x m]
            %   - Ez = Output cov [p x p]
            %   - f = State func [R^n, R^m -> R^n]
            %   - h = Output func [R^n -> R^p]
            %   - Fx = State Jacobian func [Rn, Rm -> Rnn]
            %   - Fu = Input Jacobian func [Rn, Rm -> Rnm]
            %   - Hx = Output Jacobian func [Rn -> Rpn]
            obj@kalman.AbsKF(xh, Ex, Eu, Ez);
            obj.f = f;
            obj.h = h;
            obj.Fx_ = Fx;
            obj.Fu_ = Fu;
            obj.Hx_ = Hx;
        end
        
        function xh = predict(obj, u)
            %xh = PREDICT(obj, u)
            %   Prediction step
            %   - u = Input vector [m x 1]
            %   - xh = Predicted state [n x 1]
            obj.Fx = obj.Fx_(obj.xh, u);
            obj.Fu = obj.Fu_(obj.xh, u);
            xh = predict@kalman.AbsKF(obj, u);
        end
        
        function xh = correct(obj, z)
            %xh = CORRECT(obj, z)
            %   Correction step
            %   - z = Output vector [p x 1]
            %   - xh = Corrected state [n x 1]
            obj.Hx = obj.Hx_(obj.xh);
            xh = correct@kalman.AbsKF(obj, z);
        end
    end
    
    methods (Access = protected)
        function x = predict_x(obj, u)
            %x = PREDICT_X(obj, u)
            %   Predict state
            %   - u = Input vector [m x 1]
            %   - x = Predicted state [n x 1]
            x = obj.f(obj.xh, u);
        end
        
        function z = predict_z(obj)
            %z = PREDICT_Z(obj, x)
            %   Predict output
            %   - z = Predicted output [p x 1]
            z = obj.h(obj.xh);
        end
    end
end