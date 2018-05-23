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
        
        function [nextState, varargout] = propagate(this, currentState, input, duration, varargin)
%             tic
            validateattributes(currentState, {'double'},{'numel',6,'nonempty'},'Forward propagete', 'currentState', 1);
            validateattributes(input,{'double'},{'numel',2,'nonempty'},'Forward propagete', 'input', 2);
            validateattributes(duration,{'double'},{'positive','<=',10,'>=',this.StepSize,'nonempty'},'Forward propagete', 'duration', 3);
            
            parser = inputParser;
            parser.FunctionName = "vehicle propagate";
            parser.addParameter('Log', 0);
            parser.parse(varargin{:});
            LogState =  parser.Results.Log;
            
            %Runge-Kutta integration
            if(LogState)
                [nextState, log, t] = this.RungeKutta(currentState, input, duration, true);
                varargout{1} = log;
                varargout{2} = t;
            else
                nextState = this.RungeKutta(currentState, input, duration, 0);
            end
%             toc
        end
        
        function [endState, varargout] = RungeKutta(this, startState, input, duration, LogState)
            
            stepSize = this.StepSize;
            
            %num of iterations
            numIte = floor(duration / stepSize);
            lastStepSize = rem(duration, stepSize);
            
            if(LogState)
                log = startState;
                t = 0;
            end
            
            state = startState;
            for i = 1 : (numIte + 1)
                if(i == (numIte + 1))
                    stepSize = lastStepSize;
                end
                
                if(stepSize == 0)
                    break;
                end
                k1 = this.diffEquations(state, input);
                k2 = this.diffEquations(state + 0.5 * stepSize * k1, input);
                k3 = this.diffEquations(state + 0.5 * stepSize * k2, input);
                k4 = this.diffEquations(state + stepSize * k3, input);
                
                state = state + (stepSize / 6) * (k1 + 2*k2 + 2*k3 + k4);
                
                if(LogState)
                    log(i,:) = state;
                    t(i) = i * stepSize; %#ok<AGROW>
                end
%                 diff_state = (stepSize / 6) * (k1 + 2*k2 + 2*k3 + k4);
%                 fprintf("diff_state: \n XPosition: %f;  YPosition: %f; YawAngle: %f; \n XVelocity: %f YVelocity: %f YawRate: %f\n", ...
%                         diff_state(1), diff_state(2),diff_state(3) * 180/pi, diff_state(4),diff_state(5),diff_state(6) * 180/pi);
            end
            endState = state;
            if(LogState)
                varargout{1} = log;
                varargout{2} = t;
            else
                varargout{1} = 0;
                varargout{2} = 0;
            end
%              fprintf("endState: \n XPosition: %f;  YPosition: %f; YawAngle: %f; \n XVelocity: %f YVelocity: %f YawRate: %f\n", ...
%                         endState(1), endState(2),endState(3) * 180/pi, endState(4),endState(5),endState(6) * 180/pi);
        end
        function Fy = lateralForce(this, alpha)
            r = 0;
            Fz = 5;
            a0 = 1.65;
            a1 = -34;
            a2 = 1250;
            a3 = 3036;
            a4 = 12.8;
            a5 = 0.00501;
            a6 = -0.02103;
            a7 = 0.77394;
            a8 = 0.0022890;
            a9 = 0.013442;
            a10 = 0.003709;
            a11 = 19.1656;
            a12 = 1.21356;
            a13 = 6.26206;
            C = a0;
            D = a1 * Fz^2 + a2 * Fz;
            BCD = a3 * sin(2 * atan(Fz/a4)) * (1 - a5 * abs(r));
            B = BCD / (C * D);
            Sh = a9 * Fz + a10 + a8 * r;
            k = alpha + Sh;
            Sv = a11 * Fz * r + a12 * Fz + a13;
            E = a6 * Fz^2 + a7;
            
            Fy = D * sin(C * atan(B * k - E * (B * k - atan(B * k)))) + Sv;
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
            
            m   = 1600;             % [kg]      vehicle mass
            Jz  = 2200;             % [kg m2]   yaw moment of inertia
            l   = 2.7;              % [m]       wheelbase
            lf  = 1;                % [m]       distance of c.o.g. from front axle
            lr  = l-lf;             % [m]       distance of c.o.g. from rear axle
            Fzf = m*g*lr/l;
            Fzr = m*g*lf/l;
            gT  = 0;                % traction ratio: 1 FWD, 0 RWD, 0.5 4WD (50:50)
            gB  = 2/3;                % brake ratio: F/F+R
            
            % resistance forces
            fv      = .02;        			% []		rolling resistance coeff.
            rho 	= 1.2;					% [kg/m3]	air density
            Cd  	= .3;					% []		drag coefficient
            S   	= 2;					% [m2]		front surface
            
            % lateral force MF Tire parameters
            bf = 12;        %       cornering factor
            Cf = 1.3;       %       asymptotic factor
            df = 0.95;      %       peak factor
            Ef = -0.1;       %       shape factor
            br = 14;
            Cr = 1.3;
            dr = 1;
            Er = -0.1;
            L  = 2;         % [m]   relaxation lenght
            mu = 1;
            
            x   = state(1);
            y   = state(2);
            psi = state(3); % Yaw angle
            Vx  = state(4);
            Vy  = state(5);
            omega = state(6); % Yaw rate
            
            delta = input(1); % Steering angle
            Frl = input(2);   % Force on rear wheels
            
            %slip angle 
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
            
            %lateral force
            Ffc = Cf * alphaF;            
            Frc = Cr * alphaR;
%             Ffc = this.lateralForce(alphaF * 180/pi) / 2;
%             Frc = this.lateralForce(alphaR * 180/pi) / 2;
            
            Frlimit =  m * g * mu * 0.25 + 1500;
            if(abs(Ffc) > Frlimit)
                Ffc = Frlimit * sign(Ffc);
            end
            if(abs(Frc) > Frlimit)
                Frc = Frlimit * sign(Frc);
            end
%             fprintf("alphaF: %f;  alphaR: %f;\n Ffc: %f;  Frc: %f\n", alphaF * 180 / pi, alphaR * 180 / pi, Ffc, Frc);
           
            %aerodynamic resistance 
            Af = this.AF; % Longitudinal drag area (m^2)
            Cd = this.CD; % Logitudinal drag coefficient (-)
            rho = 1.29;   % Density of air (kg / m^3)
            Faero = 0.5 * rho * Vx^2 * Cd * Af;
     
            x_diff = Vx * cos(psi) - Vy * sin(psi); % X position
            y_diff = Vx * sin(psi) + Vy * cos(psi); % Y position 
            psi_diff = omega;                       % Yaw angle
            Vx_diff = omega * Vy + Frl / m - 2 * Ffc * sin(delta) / m - Faero / m; % Logitude speed
            Vy_diff = -omega * Vx + 2 * Frc / m + 2 * Ffc * cos(delta) / m; % Lateral speed
            omega_diff = (2 / Iz) * (-Frc * lr + Ffc * lf * cos(delta)); % Yaw rate
            
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

