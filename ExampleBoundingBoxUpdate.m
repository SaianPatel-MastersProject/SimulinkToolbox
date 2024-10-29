function ExampleBoundingBoxUpdate(block, ~)
% This is a helper function for the Simulink library bounding box example.

u0 = block.InputPort(1).Data(1:3);
u1 = block.InputPort(1).Data(4:end);

ego = evalin('base', 'egoBox');
bpg = evalin('base', 'boxPlotGlobal');
bpl = evalin('base', 'boxPlotLocal');

verts = [-1,-1,0; -1,-1,1; -1,1,1; -1,1,0; 1,-1,0; 1,-1,1; 1,1,1; 1,1,0];
offset = ones(8,1)*u0';
set(ego, 'Vertices', verts + offset);

for iter = 1:length(bpg)
    set(bpg(iter), 'Vertices', reshape(u1((1:24) + 24*(iter-1)), 3, [])');
    set(bpl(iter), 'Vertices', reshape(u1((1:24) + 24*(iter-1)), 3, [])' - offset);
end
