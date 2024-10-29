% Add Physics Library to Simulink Library Browser
function blkStruct = slblocks

blkStruct.Name  = 'rFpro Physics';
Browser.Library = 'rFproPhysicsLib';
Browser.Name    = 'rFpro Physics';
blkStruct.Browser = Browser;
