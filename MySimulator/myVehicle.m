classdef myVehicle
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        % Total mass of vehicle   (kg)
        M       = 1575;                    
        % Yaw moment of inertia of vehicle   (m*N*s^2)    
        IZ      = 2875;            
        % Longitudinal distance from c.g. to front tires (m)
        LF      = 1.2;      
         % Longitudinal distance from c.g. to rear tires  (m)
        LR      = 1.6;     
         % Cornering stiffness of front tires             (N/rad)
        CF      = 19000;  
        % Cornering stiffness of rear tires              (N/rad)
        CR      = 33000;   
        % Longitudinal drag area (m^2)
        AF      = 2;
        % Logitudinal drag coefficient (-)
        CD      = 0.3;
        % Absolute pressure (Pa)
        PABS    = 101325;
        % Air temperature (K)
        K       = 273;
        % Gravitational acceleration (m/s^2)
        G       = 9.81;
        % Nominal friction scaling factor (-)
        MU      = 1;
        % Step size (s)
        StepSize = 0.02;
    end
    
    methods
        function this = myVehicle(varargin)
            
            inputs = this.parseInputs(varargin{:});
            
            this.M  = inputs.M;
            this.IZ = inputs.IZ;
            this.LF = inputs.LF;
            this.LR = inputs.LR;
            this.CF = inputs.CF;
            this.CR = inputs.CR;
            this.CD = inputs.CD;
            this.AF = inputs.AF;
            this.PABS = inputs.PABS;
            this.G  = inputs.G;
            this.K  = inputs.K;
            this.MU = inputs.MU;
        end
        
        function nextState = propagate(this, currentState, input, duration)
%             tic
            validateattributes(currentState, {'double'},{'numel',6,'nonempty'},'Forward propagete', 'currentState', 1);
            validateattributes(input,{'double'},{'numel',2,'nonempty'},'Forward propagete', 'input', 2);
            validateattributes(duration,{'double'},{'positive','<=',10,'>=',this.StepSize,'nonempty'},'Forward propagete', 'duration', 3);

            %Runge-Kutta integration
            nextState = this.RungeKutta(currentState, input, duration);
%             toc
        end
        
        function endState = RungeKutta(this, startState, input, duration)
            
            stepSize = this.StepSize;
            
            %num of iterations
            numIte = floor(duration / stepSize);
            lastStepSize = rem(duration, stepSize);
            
            state = startState;
            for i = 1 : (numIte + 1)
                if(i == (numIte + 1))
                    stepSize = lastStepSize;
                end
                k1 = this.diffEquations(state, input);
                k2 = this.diffEquations(state + 0.5 * stepSize * k1, input);
                k3 = this.diffEquations(state + 0.5 * stepSize * k2, input);
                k4 = this.diffEquations(state + stepSize * k3, input);
                diff_state = (stepSize / 6) * (k1 + 2*k2 + 2*k3 + k4);
                state = state + (stepSize / 6) * (k1 + 2*k2 + 2*k3 + k4);
%                 fprintf("diff_state: \n XPosition: %f;  YPosition: %f; YawAngle: %f; \n XVelocity: %f YVelocity: %f YawRate: %f\n", ...
%                         diff_state(1), diff_state(2),diff_state(3) * 180/pi, diff_state(4),diff_state(5),diff_state(6) * 180/pi);
            end
            endState = state;
%              fprintf("endState: \n XPosition: %f;  YPosition: %f; YawAngle: %f; \n XVelocity: %f YVelocity: %f YawRate: %f\n", ...
%                         endState(1), endState(2),endState(3) * 180/pi, endState(4),endState(5),endState(6) * 180/pi);
        end
    end
    
    methods (Access = private)
        %------------------------------------------------------------------
        function inputs = parseInputs(this, varargin)
            
            parser = inputParser;
            parser.FunctionName = mfilename;
            
            parser.addParameter('M', this.M);
            parser.addParameter('IZ', this.IZ);
            parser.addParameter('LF', this.LF);
            parser.addParameter('LR', this.LR);
            parser.addParameter('CF', this.CF);
            parser.addParameter('CR', this.CR);
            parser.addParameter('AF', this.AF);
            parser.addParameter('CD', this.CD);
            parser.addParameter('PABS', this.PABS);
            parser.addParameter('K', this.K);
            parser.addParameter('G', this.G);
            parser.addParameter('MU', this.MU);
            parser.addParameter('StepSize', this.StepSize);
           
            parser.parse(varargin{:});
            
            inputs = parser.Results;
        end
        
        function X_diff = diffEquations(this, state, input)
            
            m  = this.M;
            Iz = this.IZ;
            lf = this.LF;
            lr = this.LR;
            Cf = this.CF;
            Cr = this.CR;
            mu = this.MU;
            g = this.G;
            
            x   = state(1);
            y   = state(2);
            psi = state(3);
            Vx  = state(4);
            Vy  = state(5);
            omega = state(6);
            
            delta = input(1);
            Frl = input(2);
            
            %slip angle - todo
            if(Vx ~= 0)
                alphaF = delta - atan((Vy + lf * omega)/Vx) ;
            else
                alphaF = 0;
            end
            if(Vx ~= 0)
                alphaR = - atan((Vy - lr * omega)/Vx);
            else 
                alphaR = 0;
            end
            
%             alphaLimit = 9;
%             if(abs(alphaF) > (pi * alphaLimit / 180))
%                 alphaF = (pi * alphaLimit / 180) * sign(alphaF);
%             end
%             if(abs(alphaR) > (pi * alphaLimit / 180))
%                 alphaR = (pi * alphaLimit / 180) * sign(alphaR);
%             end
            
            %lateral force - todo
            Ffc = Cf * alphaF;            
            Frc = Cr * alphaR;
            
            Frlimit =  m * g * mu * 0.25 + 1500;
            if(abs(Ffc) > Frlimit)
                Ffc = Frlimit * sign(Ffc);
            end
            if(abs(Frc) > Frlimit)
                Frc = Frlimit * sign(Frc);
            end
%             fprintf("alphaF: %f;  alphaR: %f;\n Ffc: %f;  Frc: %f\n", alphaF * 180 / pi, alphaR * 180 / pi, Ffc, Frc);
           
            %aerodynamic resistance - todo
            Af = this.AF;
            Cd = this.CD;
            rho = 1.29;
            Faero = 0.5 * rho * Vx^2 * Cd * Af;
            
            Flimit = m * g * mu * 0.5;
            if(abs(Frl) > Flimit)
                Frl = Flimit * sign(Frl);
            end
            Frl = Frl * 0.95;
            x_diff = Vx * cos(psi) - Vy * sin(psi);
            y_diff = Vx * sin(psi) + Vy * cos(psi);
            psi_diff = omega;
            Vx_diff = omega * Vy + Frl / m - 2 * Ffc * sin(delta) / m - Faero / m;
            Vy_diff = -omega * Vx + 2 * Frc / m + 2 * Ffc * cos(delta) / m;
            omega_diff = (2 / Iz) * (-Frc * lr + Ffc * lf * cos(delta));
            
%             %test
%             x_diff = x;
%             y_diff = y;
%             psi_diff = psi;
%             Vx_diff = Vx;
%             Vy_diff = Vy;
%             omega_diff = omega;

            X_diff = [x_diff y_diff psi_diff Vx_diff Vy_diff omega_diff];
            
            if(any(isnan(X_diff) | isinf(X_diff)))
                error('X_diff  == (NaN || inf) ! ');  
            end
        end
        
        
    end
end

