%% Set xRef and yRef
% AIW_Table = readtable('+PostProcessing\+CTE\Arrow_IP.csv');
% AIW_Table = readtable('+PostProcessing\+CTE\2kF_Circ_100_CCW.csv');
% AIW_Table = readtable('+PostProcessing\+CTE\2kF_SUZE9.csv');
AIW_Table = Utilities.fnLoadAIW('SUZ_MR');
xRefOriginal = [AIW_Table.x];
yRefOriginal = [AIW_Table.y];
[kappa, ~] = PostProcessing.PE.fnCalculateCurvature([xRefOriginal, yRefOriginal]);


%% Interpolate
xRef = Utilities.fnInterpolateByDist([xRefOriginal, yRefOriginal], xRefOriginal, 1, 'spline');
yRef = Utilities.fnInterpolateByDist([xRefOriginal, yRefOriginal], yRefOriginal, 1, 'spline');
kappaInterp = Utilities.fnInterpolateByDist([xRefOriginal, yRefOriginal], kappa, 1, 'spline');

% %% Test interpolation
% dBetweenPoints = (sqrt(diff(xRef).^2 + diff(yRef).^2));
% rollingDistance = [0; cumsum(dBetweenPoints)];
% dNew = (linspace(0, rollingDistance(end), 10000))';
% 
% xInterp = interp1(rollingDistance, xRef, dNew, 'spline');
% yInterp = interp1(rollingDistance, yRef, dNew, 'spline');