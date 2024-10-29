% Check script for TestModel_OpponentTelemetry_2kFlat

load('TestModel_OpponentTelemetry_2kFlat_output.mat')

plugin.id1 = data(2,:);
plugin.id2 = data(3,:);
plugin.idx = (plugin.id1 >=0) & (plugin.id2 >= 0) & data(6,:) > -1;

plugin.t = data(1, plugin.idx)';

plugin.x1 = data(4, plugin.idx)';
plugin.y1 = data(5, plugin.idx)';
plugin.z1 = data(6, plugin.idx)';

plugin.x2 = data(7, plugin.idx)';
plugin.y2 = data(8, plugin.idx)';
plugin.z2 = data(9, plugin.idx)';

plugin.vx1 = data(10, plugin.idx)';
plugin.vy1 = data(11, plugin.idx)';
plugin.vz1 = data(12, plugin.idx)';

plugin.vx2 = data(13, plugin.idx)';
plugin.vy2 = data(14, plugin.idx)';
plugin.vz2 = data(15, plugin.idx)';

subplot(2,2,1)
plot(plugin.t, plugin.x1, 'b', plugin.t, plugin.x2, 'r')
title('X Position')

subplot(2,2,2)
plot(plugin.t, plugin.y1, 'b', plugin.t, plugin.y2, 'r')
title('Y Position')

subplot(2,2,3)
plot(plugin.t, plugin.vx1, 'b', plugin.t, plugin.vx2, 'r')
title('X Velocity')

subplot(2,2,4)
plot(plugin.x1, plugin.y1, 'b', plugin.x2, plugin.y2, 'r')
title('Trajectories')


