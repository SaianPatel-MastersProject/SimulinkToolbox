%% Set xRef and yRef
AIW_Table = Utilities.fnLoadAIW('SUZ_R');
xRefOriginal = [AIW_Table.x];
yRefOriginal = [AIW_Table.y];
[kappa, ~] = PostProcessing.PE.fnCalculateCurvature([xRefOriginal, yRefOriginal]);


%% Interpolate
xRef = Utilities.fnInterpolateByDist([xRefOriginal, yRefOriginal], xRefOriginal, 0.1, 'spline');
yRef = Utilities.fnInterpolateByDist([xRefOriginal, yRefOriginal], yRefOriginal, 0.1, 'spline');
kappaInterp = Utilities.fnInterpolateByDist([xRefOriginal, yRefOriginal], kappa, 0.1, 'spline');

%% Test interpolation
% dBetweenPoints = (sqrt(diff(xRef).^2 + diff(yRef).^2));
% rollingDistance = [0; cumsum(dBetweenPoints)];
% dNew = (linspace(0, rollingDistance(end), 10000))';
% 
% xInterp = interp1(rollingDistance, xRef, dNew, 'spline');
% yInterp = interp1(rollingDistance, yRef, dNew, 'spline');