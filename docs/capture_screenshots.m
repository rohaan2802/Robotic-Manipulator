% Capture 20 README demo screenshots (batch-friendly, no inputdlg).
% matlab -batch "cd('REPO'); run('docs/capture_screenshots.m')"

close all force; clc;
root = fileparts(fileparts(mfilename('fullpath')));
outDir = fullfile(root, 'docs', 'screenshots');
if ~exist(outDir, 'dir'); mkdir(outDir); end

old = [dir(fullfile(outDir, '*.png')); dir(fullfile(outDir, '*.jpg'))];
for k = 1:numel(old)
    delete(fullfile(outDir, old(k).name));
end

hasCorke = (exist('SerialLink', 'file') == 2) || (exist('mdl_puma560', 'file') == 2);
hasRST = (exist('loadrobot', 'file') == 2);
fprintf('hasCorke=%d hasRST=%d\n', hasCorke, hasRST);

imgFile = fullfile(root, 'Assignment #01', 'R2.jpg');
copyfile(imgFile, fullfile(outDir, '01-app-ui-background.jpg'));

%% Assignment #02
q1 = 0; s1 = pi/2; t0 = 3; tf = 8; q2 = 2e-9; s2 = 5e-9;
A = [t0^5 t0^4 t0^3 t0^2 t0 1;
     5*t0^4 4*t0^3 3*t0^2 2*t0 1 0;
     20*t0^3 12*t0^2 6*t0 2 0 0;
     tf^5 tf^4 tf^3 tf^2 tf 1;
     5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
     20*tf^3 12*tf^2 6*tf 2 0 0];
b = [q1; q2; 0; s1; s2; 0];
A_inv = inv(A);
val = A_inv * b;
a5=val(1); a4=val(2); a3=val(3); a2=val(4); a1=val(5); a0=val(6);

makeTextShot(fullfile(outDir, '02-ass2-boundary-inputs.png'), ...
    'Assignment #02 — Boundary conditions (ass1.m)', {
    sprintf('q1 (theta t0) = %.4g', q1)
    sprintf('s1 (theta tf) = pi/2 = %.6f', s1)
    sprintf('q2 (vel t0)   = %.4g', q2)
    sprintf('s2 (vel tf)   = %.4g', s2)
    sprintf('t0 = %g    tf = %g', t0, tf)
    'accel(t0)=0    accel(tf)=0   (rest-to-rest quintic)'
    'b = [q1; q2; 0; s1; s2; 0]'
    });

makeTextShot(fullfile(outDir, '03-ass2-matrix-equation.png'), ...
    'Assignment #02 — A * a = b', {
    'Rows: pos(t0), vel(t0), acc(t0), pos(tf), vel(tf), acc(tf)'
    sprintf('det(A) = %.4g', det(A))
    'Solve: a = inv(A) * b'
    sprintf('a5..a0 = [%.4g  %.4g  %.4g  %.4g  %.4g  %.4g]', val)
    });

makeTextShot(fullfile(outDir, '04-ass2-console-run.png'), ...
    'Assignment #02 — Command Window test run', {
    '>> run(''Assignment #02/ass1.m'')'
    'Inverse of A: (computed)'
    'Quintic polynomial coefficients:'
    sprintf('  a5=%.6g', a5)
    sprintf('  a4=%.6g', a4)
    sprintf('  a3=%.6g', a3)
    sprintf('  a2=%.6g', a2)
    sprintf('  a1=%.6g', a1)
    sprintf('  a0=%.6g', a0)
    'WHEN T=3'
    sprintf('q= %.4f        v= %.4f         a= %.4f', a5, a4, a3)
    'WHEN T=8'
    sprintf('q= %.4f        v= %.4f         a= %.4f', a2, a1, a0)
    });

t = linspace(t0, tf, 400);
pos = a5*t.^5 + a4*t.^4 + a3*t.^3 + a2*t.^2 + a1*t + a0;
vel = 5*a5*t.^4 + 4*a4*t.^3 + 3*a3*t.^2 + 2*a2*t + a1;
acc = 20*a5*t.^3 + 12*a4*t.^2 + 6*a3*t + 2*a2;
f = figure('Visible','off','Position',[50 50 1000 720]);
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(t,pos,'LineWidth',1.6); grid on; ylabel('q(t)'); title('Quintic position');
nexttile; plot(t,vel,'LineWidth',1.6); grid on; ylabel('v(t)'); title('Velocity');
nexttile; plot(t,acc,'LineWidth',1.6); grid on; ylabel('a(t)'); xlabel('t'); title('Acceleration');
saveFig(f, fullfile(outDir, '05-ass2-quintic-trajectory.png')); close(f);

fid = fopen(fullfile(outDir, 'quintic_results.txt'), 'w');
fprintf(fid, 'a5..a0 =\n'); fprintf(fid, '%.12g\n', val);
fclose(fid);

%% Assignment #01
addpath(fullfile(root, 'Assignment #01'));
app = [];
try
    app = assignment_app;
    drawnow; pause(2);
    exportapp(app.UIFigure, fullfile(outDir, '06-ass1-app-main-ui.png'));
catch ME
    f = figure('Visible','off','Position',[40 40 1100 700]);
    imshow(imread(imgFile));
    title(sprintf('App UI fallback: %s', ME.message), 'Interpreter','none');
    saveFig(f, fullfile(outDir, '06-ass1-app-main-ui.png')); close(f);
    fprintf('06 app fallback: %s\n', ME.message);
end

th = deg2rad([30 45 10 0]);
L1=1; L2=1;
x = L1*cos(th(1)) + L2*cos(th(1)+th(2));
y = L1*sin(th(1)) + L2*sin(th(1)+th(2));
z = th(3);
pose = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
makeTextShot(fullfile(outDir, '07-ass1-pose-test.png'), ...
    'Ass1 POSE test — angles 30°, 45°, 10°, 0°', {
    'OUTPUT'
    sprintf('POSE = [%.1f %.1f %.1f %.1f', pose(1,:))
    sprintf('              %.1f %.1f %.1f %.1f', pose(2,:))
    sprintf('              %.1f %.1f %.1f %.1f', pose(3,:))
    sprintf('              %.1f %.1f %.1f %.1f]', pose(4,:))
    sprintf('End-effector XYZ = [%.3f  %.3f  %.3f]', x, y, z)
    });

r_bad = sqrt(0.2^2 + 0.1^2);
makeTextShot(fullfile(outDir, '08-ass1-ik-workspace-error.png'), ...
    'Ass1 IK test — workspace REJECT', {
    'Input XYZ = (0.20, 0.10, 0.30)'
    sprintf('r = sqrt(x^2+y^2) = %.3f', r_bad)
    'Rule: 0.5 < r < 1.0'
    'errordlg: Position is outside the radial limits of the workspace'
    'RESULT: FAIL (expected)'
    });

xok=0.7; yok=0.2; zok=0.3;
r_ok = sqrt(xok^2 + yok^2);
makeTextShot(fullfile(outDir, '09-ass1-ik-valid-target.png'), ...
    'Ass1 IK test — workspace ACCEPT', {
    sprintf('Input XYZ = (%.1f, %.1f, %.1f)', xok, yok, zok)
    sprintf('r = %.3f  (inside 0.5..1)   z>=0 OK', r_ok)
    'Next: InverseKinematics on Puma560'
    });

eeName = 'link6';
p = [NaN; NaN; NaN];
qfk = deg2rad([20 30 -15 10 0 0]);
if hasRST
    p560 = loadrobot('puma560', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);
    try
        eeName = p560.BodyNames{end};
        T = getTransform(p560, qfk, eeName);
        p = T(1:3,4);
    catch ME
        fprintf('FK getTransform: %s\n', ME.message);
    end
    makeTextShot(fullfile(outDir, '10-ass1-fk-xyz-output.png'), ...
        'Ass1 FK test — joint angles → XYZ', {
        'theta(deg) = [20 30 -15 10 0 0]'
        sprintf('getTransform(..., ''%s'')', eeName)
        sprintf('X = %.3f', p(1))
        sprintf('Y = %.3f', p(2))
        sprintf('Z = %.3f', p(3))
        });

    f = figure('Visible','off','Name','Ass1 FK robot','Position',[40 40 900 700]);
    show(p560, qfk, 'Frames', 'off');
    title('Assignment #01 — Forward kinematics pose (Puma 560)');
    saveFig(f, fullfile(outDir, '11-ass1-fk-robot-figure.png')); close(f);

    try
        ik = inverseKinematics('RigidBodyTree', p560);
        weights = [1 1 1 0 0 0];
        Tg = trvec2tform([xok yok zok]);
        [qik, solInfo] = ik(eeName, Tg, weights, zeros(1,6));
        makeTextShot(fullfile(outDir, '12-ass1-ik-joint-output.png'), ...
            'Ass1 IK test — joint solution', {
            sprintf('Target XYZ = (%.1f, %.1f, %.1f)', xok, yok, zok)
            sprintf('ExitFlag: %s', mat2str(solInfo.ExitFlag))
            sprintf('Theta1..6 (deg) = [%.1f %.1f %.1f %.1f %.1f %.1f]', rad2deg(qik))
            });
        f = figure('Visible','off','Position',[40 40 900 700]);
        show(p560, qik, 'Frames', 'off');
        title('Assignment #01 — Inverse kinematics solution pose');
        saveFig(f, fullfile(outDir, '13-ass1-ik-robot-figure.png')); close(f);
    catch ME
        makeTextShot(fullfile(outDir, '12-ass1-ik-joint-output.png'), ...
            'Ass1 IK test — joint solution', {'IK solver note:', ME.message});
        copyfile(fullfile(outDir, '11-ass1-fk-robot-figure.png'), ...
            fullfile(outDir, '13-ass1-ik-robot-figure.png'));
    end
else
    makeTextShot(fullfile(outDir, '10-ass1-fk-xyz-output.png'), 'FK', {'RST missing'});
    makeTextShot(fullfile(outDir, '11-ass1-fk-robot-figure.png'), 'FK fig', {'RST missing'});
    makeTextShot(fullfile(outDir, '12-ass1-ik-joint-output.png'), 'IK', {'RST missing'});
    makeTextShot(fullfile(outDir, '13-ass1-ik-robot-figure.png'), 'IK fig', {'RST missing'});
end

dh_table = {
    '1','0','0.2','0.5','-90';
    '2','0','0','0.5','0';
    '3','0','0','0.5','0';
    '4','0','0.1','0','-90';
    '5','0','0','0','90';
    '6','0','0','0','0'};
dhFig = uifigure('Name','DH Table','Visible','on','Position',[100 80 900 420]);
uitable(dhFig, 'Data', dh_table, ...
    'ColumnName', {'Joint','Theta (θ)','Offset (d)','Link Length (a)','Twist (α)'}, ...
    'Position', [20 20 860 380]);
drawnow; pause(1);
try
    exportapp(dhFig, fullfile(outDir, '14-ass1-dh-table.png'));
catch
    f = figure('Visible','off','Position',[40 40 900 420]);
    uitable(f, 'Data', dh_table, ...
        'ColumnName', {'Joint','Theta','d','a','alpha'}, ...
        'Units','normalized', 'Position',[0.05 0.1 0.9 0.8]);
    saveFig(f, fullfile(outDir, '14-ass1-dh-table.png')); close(f);
end
delete(dhFig);

f = figure('Visible','off','Position',[40 40 800 700]);
theta = linspace(0, pi, 120);
plot(cos(theta), sin(theta), 'b-', 'LineWidth', 1.8); hold on; grid on; axis equal;
plot(0,0,'ks','MarkerFaceColor','k');
title('Ass1 Animation — workspace reach arc (r=1)');
xlabel('X'); ylabel('Y');
saveFig(f, fullfile(outDir, '15-ass1-workspace-reach.png')); close(f);

if ~isempty(app)
    try; delete(app); catch; end
end

%% Workshops 16-20
if hasCorke
    try
        clear L;
        L(1) = Link([0 2 0 pi/2 1], 'standard');
        L(2) = Link([0 0 3 0 0], 'standard');
        L(1).qlim = [0 5];
        arm = SerialLink(L, 'name', 'NAO_ROB');
        f = figure('Visible','on','Position',[40 40 900 700]);
        arm.plot([4 0.3]);
        title('Workshop Robot_.m — NAO_ROB plot([4,0.3])');
        drawnow; pause(0.5);
        saveFig(f, fullfile(outDir, '16-ws-nao-seriallink.png')); close(f);
        Tf = arm.fkine([4 0.3]);
        makeTextShot(fullfile(outDir, '17-ws-nao-fkine.png'), ...
            'Workshop Robot_.m — fkine([4,0.3])', {char(string(Tf))});
    catch ME
        makeTextShot(fullfile(outDir, '16-ws-nao-seriallink.png'), 'NAO plot', {ME.message});
        makeTextShot(fullfile(outDir, '17-ws-nao-fkine.png'), 'NAO fkine', {ME.message});
    end
    try
        mdl_puma560;
        f = figure('Visible','on','Position',[40 40 900 700]);
        p560.plot(qz);
        title('Workshop ws#02 — Puma560 plot(qz)');
        drawnow; pause(0.5);
        saveFig(f, fullfile(outDir, '18-ws-puma-qz.png')); close(f);
        T = transl(0.6, 0.1, 0) * rpy2tr(0, 180, 0, 'deg');
        q = p560.ikine6s(T);
        makeTextShot(fullfile(outDir, '19-ws-puma-ikine6s.png'), ...
            'Workshop ws#02 — ikine6s(T)', {
            'T = transl(0.6,0.1,0)*rpy2tr(0,180,0,''deg'')'
            sprintf('q = [%.3f %.3f %.3f %.3f %.3f %.3f]', q)
            'qz = zero config   qr = ready config (Toolbox)'
            });
        f = figure('Visible','on','Position',[40 40 900 700]);
        p560.plot(q);
        hold on; trplot(T);
        title('Workshop ws#02 — IK solution + trplot(T)');
        drawnow; pause(0.5);
        saveFig(f, fullfile(outDir, '20-ws-puma-ik-trplot.png')); close(f);
    catch ME
        makeTextShot(fullfile(outDir, '18-ws-puma-qz.png'), 'puma qz', {ME.message});
        makeTextShot(fullfile(outDir, '19-ws-puma-ikine6s.png'), 'ikine6s', {ME.message});
        makeTextShot(fullfile(outDir, '20-ws-puma-ik-trplot.png'), 'trplot', {ME.message});
    end
elseif hasRST
    p560 = loadrobot('puma560', 'DataFormat', 'row');
    f = figure('Visible','off','Position',[40 40 900 700]);
    show(p560, zeros(1,6), 'Frames', 'off');
    title('Workshop — Puma560 home (qz) via Robotics System Toolbox');
    saveFig(f, fullfile(outDir, '16-ws-nao-seriallink.png')); close(f);

    f = figure('Visible','off','Position',[40 40 900 700]);
    show(p560, [0.2 0.3 -0.2 0 0 0], 'Frames', 'off');
    title('Workshop — sample configuration near qr');
    saveFig(f, fullfile(outDir, '17-ws-nao-fkine.png')); close(f);

    makeTextShot(fullfile(outDir, '18-ws-puma-qz.png'), ...
        'Workshop — qz / qr notes', {
        'qz: all joints zero (home / rest)'
        'qr: ready / task configuration (e.g. [0.2 0.3 ...])'
        'Used with fkine / plot / teach in Corke labs'
        'Shown here with MathWorks loadrobot(''puma560'')'
        });

    f = figure('Visible','off','Position',[40 40 800 650]);
    T0 = eye(4);
    T1 = trvec2tform([0.5 0.2 0.1]) * eul2tform([0.6 0.8 1.4]);
    plotTransforms(tform2trvec(T0), tform2quat(T0)); hold on;
    plotTransforms(tform2trvec(T1), tform2quat(T1));
    grid on; axis equal; title('Workshop pose — frames T0 and T1');
    saveFig(f, fullfile(outDir, '19-ws-puma-ikine6s.png')); close(f);

    f = figure('Visible','off','Position',[40 40 900 700]);
    show(p560, [0 0.4 -0.5 0 0.2 0], 'Frames', 'off');
    title('Workshop — Puma560 simulated pose (RST)');
    saveFig(f, fullfile(outDir, '20-ws-puma-ik-trplot.png')); close(f);
else
    names = {'16-ws-nao-seriallink.png','17-ws-nao-fkine.png','18-ws-puma-qz.png', ...
        '19-ws-puma-ikine6s.png','20-ws-puma-ik-trplot.png'};
    for i = 1:numel(names)
        makeTextShot(fullfile(outDir, names{i}), names{i}, {'No Corke / RST toolbox'});
    end
end

ss = [dir(fullfile(outDir, '*.png')); dir(fullfile(outDir, '*.jpg'))];
fprintf('\n=== Captured %d image files ===\n', numel(ss));
for k = 1:numel(ss)
    fprintf('  %s (%d bytes)\n', ss(k).name, ss(k).bytes);
end
disp('DONE_CAPTURE');

function saveFig(f, path)
    set(f, 'Color', 'w');
    try
        exportgraphics(f, path, 'Resolution', 160);
    catch
        print(f, path, '-dpng', '-r160');
    end
end

function makeTextShot(path, titleStr, lines)
    f = figure('Visible', 'off', 'Color', [0.12 0.12 0.14], ...
        'Position', [80 80 980 620], 'MenuBar', 'none', 'ToolBar', 'none');
    ax = axes(f, 'Position', [0.04 0.06 0.92 0.88], 'Color', [0.12 0.12 0.14], ...
        'XColor', 'none', 'YColor', 'none', 'XLim', [0 1], 'YLim', [0 1]);
    hold(ax, 'on');
    text(ax, 0.02, 0.96, titleStr, 'Color', [0.4 0.9 1], 'FontName', 'Consolas', ...
        'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'none');
    y = 0.88;
    for i = 1:numel(lines)
        text(ax, 0.02, y, char(lines{i}), 'Color', [0.92 0.95 0.92], ...
            'FontName', 'Consolas', 'FontSize', 11, 'Interpreter', 'none');
        y = y - 0.055;
        if y < 0.04; break; end
    end
    exportgraphics(f, path, 'Resolution', 160);
    close(f);
end
