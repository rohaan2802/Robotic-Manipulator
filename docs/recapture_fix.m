% Re-capture failed workshop NAO shots (16, 17) + verify weak ones
root = fileparts(fileparts(mfilename('fullpath')));
outDir = fullfile(root, 'docs', 'screenshots');

% Corke NAO_ROB from Robot_.m — avoid teach (blocks)
clear L;
L(1) = Link([0 2 0 pi/2 1], 'standard');
L(2) = Link([0 0 3 0 0], 'standard');
L(1).qlim = [0 5];
arm = SerialLink(L, 'name', 'NAO_ROB');

f = figure('Visible','on','Position',[40 40 1000 750], 'Color','w');
arm.plot([4 0.3], 'workspace', [-2 6 -4 4 -1 6]);
title('Workshop Robot_.m — NAO\_ROB SerialLink plot([4, 0.3])');
drawnow; pause(1);
exportgraphics(f, fullfile(outDir, '16-ws-nao-seriallink.png'), 'Resolution', 160);
close(f);

Tf = arm.fkine([4 0.3]);
% Corke SE3 -> numeric matrix for display
try
    Tm = Tf.T;
catch
    Tm = double(Tf);
end
lines = {
    '>> arm.fkine([4, 0.3])'
    sprintf('T =')
    sprintf('  %.4f  %.4f  %.4f  %.4f', Tm(1,:))
    sprintf('  %.4f  %.4f  %.4f  %.4f', Tm(2,:))
    sprintf('  %.4f  %.4f  %.4f  %.4f', Tm(3,:))
    sprintf('  %.4f  %.4f  %.4f  %.4f', Tm(4,:))
    sprintf('Position XYZ = [%.3f  %.3f  %.3f]', Tm(1,4), Tm(2,4), Tm(3,4))
    };
makeTextShot(fullfile(outDir, '17-ws-nao-fkine.png'), ...
    'Workshop Robot_.m — fkine([4,0.3])', lines);

% Improve DH table as classic figure (exportapp missed UI chrome)
dh_table = {
    '1','0','0.2','0.5','-90';
    '2','0','0','0.5','0';
    '3','0','0','0.5','0';
    '4','0','0.1','0','-90';
    '5','0','0','0','90';
    '6','0','0','0','0'};
f = figure('Visible','off','Position',[40 40 980 420], 'Color','w', 'Name','DH Table');
uitable(f, 'Data', dh_table, ...
    'ColumnName', {'Joint','Theta (θ)','Offset (d)','Link Length (a)','Twist (α)'}, ...
    'Units','normalized', 'Position',[0.03 0.08 0.94 0.82], ...
    'FontSize', 12);
sgtitle('Assignment #01 — DH Table (Animation button)');
exportgraphics(f, fullfile(outDir, '14-ass1-dh-table.png'), 'Resolution', 160);
close(f);

% Re-run ws#02 plot+trplot if needed (keep good ones)
mdl_puma560;
T = transl(0.6, 0.1, 0) * rpy2tr(0, 180, 0, 'deg');
q = p560.ikine6s(T);
f = figure('Visible','on','Position',[40 40 1000 750], 'Color','w');
p560.plot(q);
hold on; trplot(T, 'frame', 'T', 'length', 0.25);
title('Workshop ws#02 — ikine6s solution + trplot(T)');
drawnow; pause(1);
exportgraphics(f, fullfile(outDir, '20-ws-puma-ik-trplot.png'), 'Resolution', 160);
close(f);

makeTextShot(fullfile(outDir, '19-ws-puma-ikine6s.png'), ...
    'Workshop ws#02 — ikine6s(T) test', {
    'T = transl(0.6,0.1,0)*rpy2tr(0,180,0,''deg'')'
    sprintf('q = [%.4f %.4f %.4f %.4f %.4f %.4f]', q)
    'qz = zeros (home)   qr = ready pose (Toolbox)'
    'Source: WORK_SHOPS/ws#02.m'
    });

disp('REFIX_DONE');

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
