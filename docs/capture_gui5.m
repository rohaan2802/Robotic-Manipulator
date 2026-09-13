% Capture 5 extra GUI screenshots: poses, animation, rotation limits
root = fileparts(fileparts(mfilename('fullpath')));
outDir = fullfile(root, 'docs', 'screenshots');
if ~exist(outDir, 'dir'); mkdir(outDir); end

p560 = loadrobot('puma560', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);

nJ = 0; limRows = {}; qLim = [];
for i = 1:numel(p560.Bodies)
    j = p560.Bodies{i}.Joint;
    if ~strcmp(j.Type, 'fixed')
        nJ = nJ + 1;
        lim = j.PositionLimits;
        limRows{end+1,1} = sprintf('q%d', nJ); %#ok<AGROW>
        limRows{end,2} = sprintf('%.1f deg', rad2deg(lim(1)));
        limRows{end,3} = sprintf('%.1f deg', rad2deg(lim(2)));
        limRows{end,4} = char(j.Type);
        qLim(nJ,:) = lim; %#ok<SAGROW>
    end
end
qMid = mean(qLim, 2)';
if numel(qMid) < 6; qMid = [qMid, zeros(1, 6-numel(qMid))]; end
qMid = qMid(1:6);

%% 21 — Joint rotation limits GUI (table + robot)
f = figure('Visible','on','Color',[0.94 0.95 0.97],'Position',[40 40 1280 700], ...
    'Name','Joint Rotation Limits','MenuBar','none');
uitable(f, 'Data', limRows, ...
    'ColumnName', {'Joint','Min limit','Max limit','Type'}, ...
    'Units','normalized', 'Position',[0.03 0.12 0.38 0.78], ...
    'FontSize', 12, 'ColumnWidth', {70 110 110 100});
annotation(f,'textbox',[0.03 0.91 0.38 0.06], 'String', ...
    'Puma 560 — Rotation / Joint Limits (Ass1 POSE & FK checks)', ...
    'EdgeColor','none','FontWeight','bold','FontSize',12, 'Color',[0.1 0.25 0.4]);
ax = axes(f, 'Position',[0.48 0.1 0.48 0.8]);
show(p560, qMid, 'Parent', ax, 'Frames', 'on', 'PreservePlot', false);
title(ax, 'Mid-range pose (inside rotation limits)');
drawnow; pause(1);
exportgraphics(f, fullfile(outDir, '21-gui-joint-rotation-limits.png'), 'Resolution', 150);
close(f);

%% Shared IK path for animation shots
ik = inverseKinematics('RigidBodyTree', p560);
ik.SolverParameters.AllowRandomRestart = true;
weights = [1 1 1 0.05 0.05 0.05];
ee = p560.BodyNames{end};
radius = 0.75; zArc = 0.25;

theta4 = linspace(0, pi, 4);
Qs = zeros(4,6); qGuess = zeros(1,6);
for k = 1:4
    tgt = [radius*cos(theta4(k)), radius*sin(theta4(k)), zArc];
    [q, ~] = ik(ee, trvec2tform(tgt), weights, qGuess);
    Qs(k,:) = q; qGuess = q;
end

%% 22 — Animation keyframes GUI
f = figure('Visible','on','Color','w','Position',[40 40 1200 720], 'Name','Animation keyframes');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
labels = {'Frame 1 — start','Frame 2','Frame 3','Frame 4 — end'};
for k = 1:4
    ax = nexttile;
    show(p560, Qs(k,:), 'Parent', ax, 'Frames','off','PreservePlot',false);
    title(labels{k}); view(ax, 135, 20);
end
sgtitle('Ass1-style animation — workspace reach keyframes (GUI)');
drawnow; pause(0.6);
exportgraphics(f, fullfile(outDir, '22-gui-animation-keyframes.png'), 'Resolution', 150);
close(f);

%% 23 — Path animation mid-frame
nPts = 20;
th = linspace(0, pi, nPts);
trail = [radius*cos(th); radius*sin(th); zArc*ones(1,nPts)];
qAnim = zeros(nPts,6); qGuess = zeros(1,6);
for k = 1:nPts
    [q, ~] = ik(ee, trvec2tform(trail(:,k)'), weights, qGuess);
    qAnim(k,:) = q; qGuess = q;
end
mid = round(nPts/2);
f = figure('Visible','on','Color','w','Position',[40 40 1000 750], 'Name','Reach animation');
ax = axes(f);
show(p560, qAnim(mid,:), 'Parent', ax, 'Frames','on','PreservePlot',false); hold(ax,'on');
plot3(ax, trail(1,:), trail(2,:), trail(3,:), 'm-', 'LineWidth', 2.2);
plot3(ax, trail(1,1), trail(2,1), trail(3,1), 'go', 'MarkerFaceColor','g', 'MarkerSize', 10);
plot3(ax, trail(1,end), trail(2,end), trail(3,end), 'rs', 'MarkerFaceColor','r', 'MarkerSize', 10);
title(ax, 'GUI animation — EE path (magenta) + robot mid-frame');
view(ax, 140, 18); grid(ax, 'on');
drawnow; pause(0.5);
exportgraphics(f, fullfile(outDir, '23-gui-animation-path-midframe.png'), 'Resolution', 150);
close(f);

%% 24 — Arm positions grid
poses = {
    zeros(1,6), 'Home (qz)'
    [0 0.5 -0.8 0 0.3 0], 'Ready / working'
    qAnim(end,:), 'Reach end of arc'
    [0 -pi/3 pi/2 0 pi/4 0], 'Folded / tucked'
    };
f = figure('Visible','on','Color','w','Position',[40 40 1200 720], 'Name','Arm positions');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
for k = 1:4
    ax = nexttile;
    show(p560, poses{k,1}, 'Parent', ax, 'Frames','off','PreservePlot',false);
    title(poses{k,2}); view(ax, 125, 18);
end
sgtitle('GUI — robot arm positions (demo / animation poses)');
drawnow; pause(0.5);
exportgraphics(f, fullfile(outDir, '24-gui-arm-positions-grid.png'), 'Resolution', 150);
close(f);

%% 25 — Corke NAO animation mid + qlim markers
clear L;
L(1) = Link([0 2 0 pi/2 1], 'standard');
L(2) = Link([0 0 3 0 0], 'standard');
L(1).qlim = [0 5];
L(2).qlim = [-pi pi];
arm = SerialLink(L, 'name', 'NAO_ROB');
qseq = [linspace(1,4,20)' linspace(-0.4,0.8,20)'];
f = figure('Visible','on','Color','w','Position',[40 40 1100 720], 'Name','NAO animation + limits');
arm.plot(qseq(10,:), 'workspace', [-2 6 -4 4 -1 6]);
hold on;
plot3([0 0],[0 0],[0 5], 'c--', 'LineWidth', 2.5);
text(0.25, 0, 5.15, 'q1 max = 5 (prismatic limit)', 'Color',[0 0.55 0.7], ...
    'FontSize', 11, 'FontWeight','bold');
text(0.25, 0, 0.15, 'q1 min = 0', 'Color',[0 0.55 0.7], 'FontSize', 11, 'FontWeight','bold');
text(3.2, 0.5, 2.2, 'q2 revolute lim [-pi, pi]', 'Color',[0.7 0.2 0.1], ...
    'FontSize', 11, 'FontWeight','bold');
title({sprintf('Workshop GUI animation — NAO\\_ROB mid-motion  q=[%.2f, %.2f]', ...
    qseq(10,1), qseq(10,2)); 'Rotation / stroke limits annotated'});
drawnow; pause(0.8);
exportgraphics(f, fullfile(outDir, '25-gui-nao-animation-limits.png'), 'Resolution', 150);
close(f);

ss = dir(fullfile(outDir, '2[1-5]-*.png'));
fprintf('Added %d GUI shots\n', numel(ss));
for k=1:numel(ss); fprintf('  %s (%d)\n', ss(k).name, ss(k).bytes); end
disp('GUI5_DONE');
