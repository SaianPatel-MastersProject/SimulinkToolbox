% This script assumes the data from the test model:
%   TestModel_EgoTelemetry_Barcelona.mdl
% has been loaded as data.

load('TestModel_EgoTelemetry_Barcelona_output.mat')
data = data';

plugin.valid = data(:,2) > 0; % Wait until telemetry is producing data

plugin.t_sim = data(plugin.valid,1);
plutin.t_telem = data(plugin.valid, 2);

plugin.x = data(plugin.valid, 3);
plugin.y = data(plugin.valid, 4);
plugin.z = data(plugin.valid, 5);

plugin.throttle = data(plugin.valid, 6);
plugin.brake    = data(plugin.valid, 7);
plugin.steering = data(plugin.valid, 8);
plugin.gear     = data(plugin.valid, 9);

plugin.lap_dist = data(plugin.valid, 10);

plugin.ori11 = data(plugin.valid, 11);
plugin.ori12 = data(plugin.valid, 12);
plugin.ori13 = data(plugin.valid, 13);
plugin.ori21 = data(plugin.valid, 14);
plugin.ori22 = data(plugin.valid, 15);
plugin.ori23 = data(plugin.valid, 16);
plugin.ori31 = data(plugin.valid, 17);
plugin.ori32 = data(plugin.valid, 18);
plugin.ori33 = data(plugin.valid, 19);

plugin.vx = data(plugin.valid, 20);
plugin.vy = data(plugin.valid, 21);
plugin.vz = data(plugin.valid, 22);

plugin.rx = data(plugin.valid, 23);
plugin.ry = data(plugin.valid, 24);
plugin.rz = data(plugin.valid, 25);

plugin.lap_ref = data(plugin.valid, 26);
                
% This next part assumes the model:   
%   TestModel_EgoTelemetry_Barcelona.mdl
% has been loaded into Matlab.

mdlWks = get_param('TestModel_EgoTelemetry_Barcelona', 'ModelWorkspace');
varValue = getVariable(mdlWks,'recorded_ego');
model.t = varValue.time;

model.x = varValue.signals.values(:,1);
model.y = varValue.signals.values(:,2);
model.z = varValue.signals.values(:,3);

model.ori11 = varValue.signals.values(:, 8);
model.ori12 = varValue.signals.values(:, 9);
model.ori13 = varValue.signals.values(:,10);
model.ori21 = varValue.signals.values(:,11);
model.ori22 = varValue.signals.values(:,12);
model.ori23 = varValue.signals.values(:,13);
model.ori31 = varValue.signals.values(:,14);
model.ori32 = varValue.signals.values(:,15);
model.ori33 = varValue.signals.values(:,16);

model.vx = varValue.signals.values(:,17);
model.vy = varValue.signals.values(:,18);
model.vz = varValue.signals.values(:,19);

model.rx = varValue.signals.values(:,20);
model.ry = varValue.signals.values(:,21);
model.rz = varValue.signals.values(:,22);

model.v = ((model.vx).^2 + (model.vy).^2 + (model.vz).^2).^(0.5);

% Next, we display the ouputs, waiting for a prompt from the user between
% each figure

subplot(3,3,1)
plot(model.t, model.x, 'b', plugin.t_sim, plugin.x, 'r');
title('X position')

subplot(3,3,2)
plot(model.t, model.y, 'b', plugin.t_sim, plugin.y, 'r');
title('Y position')

subplot(3,3,3)
plot(model.t, model.z, 'b', plugin.t_sim, plugin.z, 'r');
title('Z position')

% Velocities don't match because the telemetry is in local co-ordinates and
% the model is in world co-ordinates.
% *** We need a converter between local and world parameters. ***

subplot(3,3,4)
plot(model.t, model.vx, 'b', plugin.t_sim, plugin.vx, 'r', model.t, model.v, 'm');
title('X velocity')

subplot(3,3,5)
plot(model.t, model.vy, 'b', plugin.t_sim, plugin.vy, 'r');
title('Y velocity')

subplot(3,3,6)
plot(model.t, model.vz, 'b', plugin.t_sim, plugin.vz, 'r');
title('Z velocity')

subplot(3,3,7)
plot(plugin.t_sim, plugin.lap_dist, 'r');
title('Dist travelled')

subplot(3,3,8)
plot(plugin.t_sim, plugin.lap_ref, 'r');
title('Lap Position')

