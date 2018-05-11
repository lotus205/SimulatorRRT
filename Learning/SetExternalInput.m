clear
open_system('sldemo_mdlref_counter');

in = Simulink.SimulationInput('sldemo_mdlref_counter');

% t = (0:0.01:10)';
% u1 = 5*ones(size(t));
% u2 = 10*sin(t);
% u3 = -5*ones(size(t));

in.ExternalInput = [5 1 2 3];

out = sim(in);