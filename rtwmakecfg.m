% Declare additional libraries required by toolbox
% Copyright (c) rFpro Limited 2013-2020. All rights reserved.

function makeInfo = rtwmakecfg

fprintf('Running %s from folders: %s\n', mfilename, pwd);
makeInfo.includePath = { pwd };
makeInfo.sourcePath = { pwd };
if (evalin('base', 'exist(''rFproLib_IP'', ''var'')') && strcmp(evalin('base', 'rFproLib_IP.Mode'), 'Hosted Plugin'))
    makeInfo.sources  = { 'rFProPlugin.cpp' };
    fprintf('Using rFproPlugin.cpp as a source file for DLL\n')
else
    makeInfo.sources = { };
    fprintf('NOT using hosted plugin source file\n');
end
makeInfo.linkLibsObjs = { };
