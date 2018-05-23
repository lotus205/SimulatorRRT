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