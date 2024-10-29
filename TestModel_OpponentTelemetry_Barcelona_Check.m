% This script assumes the data from the test model:
%   TestModel_OpponentTelemetry_Barcelona.mdl
% has been loaded as data.

load('TestModel_OpponentTelemetry_Barcelona_output.mat')

plugin.valid = data(2,:) >= 0;

plugin.t = data(1, plugin.valid)';
plugin.x = data(3, plugin.valid)';
plugin.y = data(4, plugin.valid)';
plugin.z = data(5, plugin.valid)';

plugin.vx = data(6, plugin.valid)';
plugin.vy = data(7, plugin.valid)';
plugin.vz = data(8, plugin.valid)';

plugin.ori11 = data( 9, plugin.valid)';
plugin.ori12 = data(10, plugin.valid)';
plugin.ori13 = data(11, plugin.valid)';
plugin.ori21 = data(12, plugin.valid)';
plugin.ori22 = data(13, plugin.valid)';
plugin.ori23 = data(14, plugin.valid)';
plugin.ori31 = data(15, plugin.valid)';
plugin.ori32 = data(16, plugin.valid)';
plugin.ori33 = data(17, plugin.valid)';

plugin.rx = data(18, plugin.valid)';
plugin.ry = data(19, plugin.valid)';
plugin.rz = data(20, plugin.valid)';

plugin.beacon = data(21:26, plugin.valid)';
plugin.beacon_value =  1* plugin.beacon(:,1) +  2 * plugin.beacon(:,2) ...
                    +  4 * plugin.beacon(:,3) +  8 * plugin.beacon(:,4) ...
                    + 16 * plugin.beacon(:,3) + 32 * plugin.beacon(:,4);

                
% This next part assumes the model:   
%   TestModel_OpponentTelemetry_Barcelona.mdl
% has been loaded into Matlab.

mdlWks = get_param(gcs,'ModelWorkspace');
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

% Next, we display the ouputs, waiting for a prompt from the user between
% each figure

subplot(3,3,1)
plot(model.t, model.x, 'b', plugin.t, plugin.x, 'r');
title('X position')

subplot(3,3,2)
plot(model.t, model.y, 'b', plugin.t, plugin.y, 'r');
title('Y position')

subplot(3,3,3)
plot(model.t, model.z, 'b', plugin.t, plugin.z, 'r');
title('Z position')

subplot(3,3,4)
plot(model.t, model.vx, 'b', plugin.t, plugin.vx, 'r');
title('X velocity')

subplot(3,3,5)
plot(model.t, model.vy, 'b', plugin.t, plugin.vy, 'r');
title('Y velocity')

subplot(3,3,6)
plot(model.t, model.vz, 'b', plugin.t, plugin.vz, 'r');
title('Z velocity')

subplot(3,3,7)
plot(plugin.t, plugin.beacon_value, 'b');
title('Beacon values')
