% HRVO step-by-step explanation demo (for teaching/slides)
% Run:
%   hrvo_step_by_step_demo
%
% This script animates 6 steps:
% 1) Position geometry
% 2) Collision cone (relative velocity idea)
% 3) VO in velocity space
% 4) RVO in velocity space
% 5) HRVO in velocity space
% 6) Pick vSelected closest to vPref outside forbidden region

clear; clc; close all;

%% Scenario
pA = [-1.8, 0.0];
pB = [ 1.8, 0.35];
vA = [ 0.45, 0.02];
vB = [-0.35, -0.03];
rA = 0.35;
rB = 0.35;
vPrefA = [0.95, 0.00];
vMax = 1.30;
combinedRadius = rA + rB;

% Build all constraints once
cVO = buildConstraint(pA, vA, pB, vB, combinedRadius, "VO");
cRVO = buildConstraint(pA, vA, pB, vB, combinedRadius, "RVO");
cHRVO = buildConstraint(pA, vA, pB, vB, combinedRadius, "HRVO");
[vSelVO, ~] = chooseVelocity(vPrefA, cVO, vMax);
[vSelRVO, ~] = chooseVelocity(vPrefA, cRVO, vMax);
[vSelHRVO, ~] = chooseVelocity(vPrefA, cHRVO, vMax);

fig = figure('Color', 'w', 'Name', 'HRVO Step-by-Step Demo');
set(fig, 'Position', [80 80 1260 620]);

axL = axes('Position', [0.06 0.14 0.39 0.76]); %#ok<LAXES>
axR = axes('Position', [0.55 0.14 0.39 0.76]); %#ok<LAXES>

for step = 1:6
    cla(axL); cla(axR);
    drawLeftPanel(axL, pA, pB, vA, vB, rA, rB, step);
    drawRightPanel(axR, cVO, cRVO, cHRVO, vA, vB, vPrefA, ...
        vSelVO, vSelRVO, vSelHRVO, step);

    annotation('textbox', [0.02 0.93 0.96 0.06], 'String', stepMessage(step), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontWeight', 'bold', 'FontSize', 12, 'EdgeColor', 'none');

    drawnow;
    pause(1.35);
end

disp('Done. Step-by-step HRVO explanation finished.');

function drawLeftPanel(ax, pA, pB, vA, vB, rA, rB, step)
hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
title(ax, 'Position space (x-y)', 'FontWeight', 'bold');
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');

pad = 0.9;
xmin = min([pA(1)-rA, pB(1)-rB]) - pad;
xmax = max([pA(1)+rA, pB(1)+rB]) + pad;
ymin = min([pA(2)-rA, pB(2)-rB]) - pad;
ymax = max([pA(2)+rA, pB(2)+rB]) + pad;
xlim(ax, [xmin xmax]); ylim(ax, [ymin ymax]);

th = linspace(0, 2*pi, 200);
ca = pA + rA * [cos(th(:)), sin(th(:))];
cb = pB + rB * [cos(th(:)), sin(th(:))];
fill(ax, ca(:,1), ca(:,2), [0.18 0.55 0.95], 'FaceAlpha', 0.32, ...
    'EdgeColor', [0.08 0.3 0.75], 'LineWidth', 1.6);
fill(ax, cb(:,1), cb(:,2), [0.95 0.43 0.25], 'FaceAlpha', 0.32, ...
    'EdgeColor', [0.8 0.18 0.08], 'LineWidth', 1.6);
plot(ax, pA(1), pA(2), 'o', 'MarkerFaceColor', [0.08 0.3 0.75], 'Color', [0.08 0.3 0.75]);
plot(ax, pB(1), pB(2), 'o', 'MarkerFaceColor', [0.8 0.18 0.08], 'Color', [0.8 0.18 0.08]);
text(ax, pA(1), pA(2)-0.14, 'A', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(ax, pB(1), pB(2)-0.14, 'B', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

plot(ax, [pA(1), pB(1)], [pA(2), pB(2)], 'k--', 'LineWidth', 1.2);
mid = 0.5*(pA+pB);
d = norm(pB - pA);
text(ax, mid(1), mid(2)+0.12, sprintf('d = %.2f m', d), ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
text(ax, mid(1), mid(2)-0.12, sprintf('collision threshold = rA+rB = %.2f m', rA+rB), ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);

if step >= 2
    quiver(ax, pA(1), pA(2), vA(1), vA(2), 0, 'Color', 'k', 'LineWidth', 1.8, 'MaxHeadSize', 0.8);
    quiver(ax, pB(1), pB(2), vB(1), vB(2), 0, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.8, 'MaxHeadSize', 0.8);
    text(ax, pA(1)+vA(1), pA(2)+vA(2)+0.06, 'vA', 'FontWeight', 'bold');
    text(ax, pB(1)+vB(1), pB(2)+vB(2)+0.06, 'vB', 'FontWeight', 'bold');
end
end

function drawRightPanel(ax, cVO, cRVO, cHRVO, vA, vB, vPrefA, vSelVO, vSelRVO, vSelHRVO, step)
hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
title(ax, 'Velocity space (v_x-v_y)', 'FontWeight', 'bold');
xlabel(ax, 'v_x (m/s)'); ylabel(ax, 'v_y (m/s)');
xlim(ax, [-1.4, 1.6]); ylim(ax, [-1.2, 1.2]);

switch step
    case 1
        text(ax, -1.2, 0, 'Step 1: chưa dựng nón trong velocity space', 'FontWeight', 'bold');
    case 2
        % Relative collision cone shown at origin for intuition.
        cRel = cVO;
        cRel.apex = [0, 0];
        drawConePatch(ax, cRel, 2.5, [0.55 0.1 0.75], 'relative collision cone');
    case 3
        drawConePatch(ax, cVO, 2.5, [0.2 0.55 0.95], 'VO forbidden');
        plotCommonVelocityPoints(ax, vA, vB, vPrefA);
        plot(ax, vSelVO(1), vSelVO(2), 'd', 'Color', [0.2 0.55 0.95], ...
            'MarkerFaceColor', [0.2 0.55 0.95], 'DisplayName', 'vSelected-VO');
    case 4
        drawConePatch(ax, cRVO, 2.5, [0.12 0.7 0.45], 'RVO forbidden');
        plotCommonVelocityPoints(ax, vA, vB, vPrefA);
        plot(ax, vSelRVO(1), vSelRVO(2), 'd', 'Color', [0.12 0.7 0.45], ...
            'MarkerFaceColor', [0.12 0.7 0.45], 'DisplayName', 'vSelected-RVO');
    case 5
        drawConePatch(ax, cHRVO, 2.5, [0.95 0.45 0.15], 'HRVO forbidden');
        plotCommonVelocityPoints(ax, vA, vB, vPrefA);
        plot(ax, vSelHRVO(1), vSelHRVO(2), 'd', 'Color', [0.95 0.45 0.15], ...
            'MarkerFaceColor', [0.95 0.45 0.15], 'DisplayName', 'vSelected-HRVO');
    otherwise
        drawConePatch(ax, cVO, 2.5, [0.2 0.55 0.95], 'VO');
        drawConePatch(ax, cRVO, 2.5, [0.12 0.7 0.45], 'RVO');
        drawConePatch(ax, cHRVO, 2.5, [0.95 0.45 0.15], 'HRVO');
        plotCommonVelocityPoints(ax, vA, vB, vPrefA);
        plot(ax, vSelVO(1), vSelVO(2), 'd', 'Color', [0.2 0.55 0.95], 'MarkerFaceColor', [0.2 0.55 0.95], 'DisplayName', 'vSel-VO');
        plot(ax, vSelRVO(1), vSelRVO(2), 'd', 'Color', [0.12 0.7 0.45], 'MarkerFaceColor', [0.12 0.7 0.45], 'DisplayName', 'vSel-RVO');
        plot(ax, vSelHRVO(1), vSelHRVO(2), 'd', 'Color', [0.95 0.45 0.15], 'MarkerFaceColor', [0.95 0.45 0.15], 'DisplayName', 'vSel-HRVO');
        line(ax, [vPrefA(1), vSelHRVO(1)], [vPrefA(2), vSelHRVO(2)], ...
            'Color', [0.2 0.2 0.2], 'LineStyle', '--', 'LineWidth', 1.2, 'DisplayName', 'distance to vPref');
end

if step >= 3
    annotation('textarrow', [0.46 0.54], [0.57 0.57], 'String', 'mapping', ...
        'LineWidth', 1.4, 'HeadLength', 9, 'HeadWidth', 9, 'FontWeight', 'bold');
end
legend(ax, 'Location', 'southoutside', 'NumColumns', 2);
end

function plotCommonVelocityPoints(ax, vA, vB, vPrefA)
plot(ax, vA(1), vA(2), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'vA');
plot(ax, vB(1), vB(2), 'ks', 'MarkerFaceColor', [0.35 0.35 0.35], 'DisplayName', 'vB');
plot(ax, vPrefA(1), vPrefA(2), 'kp', 'MarkerFaceColor', [0.08 0.7 0.1], 'DisplayName', 'vPrefA');
end

function msg = stepMessage(step)
switch step
    case 1
        msg = 'Step 1/6: Xac dinh hinh hoc trong position space: pA, pB, rA, rB.';
    case 2
        msg = 'Step 2/6: Tu vi tri + kich thuoc -> collision cone trong relative velocity.';
    case 3
        msg = 'Step 3/6: Dich cone den apex = vB -> VO.';
    case 4
        msg = 'Step 4/6: Dich cone den apex = (vA+vB)/2 -> RVO.';
    case 5
        msg = 'Step 5/6: Lai VO + RVO theo quy tac ben trai/phai -> HRVO.';
    otherwise
        msg = 'Step 6/6: Chon vSelected gan vPref nhat nhung nam ngoai vung cam.';
end
end

function c = buildConstraint(pA, vA, pB, vB, combinedRadius, method)
relPos = pB - pA;
dist = norm(relPos);
if dist < 1e-9
    relPos = [1, 0];
    dist = 1;
end

theta = atan2(relPos(2), relPos(1));
alpha = asin(min(0.999, combinedRadius / max(dist, combinedRadius + 1e-6)));
leftDir = [cos(theta + alpha), sin(theta + alpha)];
rightDir = [cos(theta - alpha), sin(theta - alpha)];

apexVO = vB;
apexRVO = 0.5 * (vA + vB);

switch upper(method)
    case "VO"
        c.apex = apexVO;
        c.leftDir = leftDir;
        c.rightDir = rightDir;
    case "RVO"
        c.apex = apexRVO;
        c.leftDir = leftDir;
        c.rightDir = rightDir;
    otherwise
        centerDir = relPos / dist;
        side = cross2d(centerDir, vA - apexRVO);
        if side >= 0
            [ok, apexH] = lineIntersection(apexRVO, leftDir, apexVO, rightDir);
            if ~ok, apexH = apexRVO; end
            c.apex = apexH;
            c.leftDir = leftDir;
            c.rightDir = rightDir;
        else
            [ok, apexH] = lineIntersection(apexVO, leftDir, apexRVO, rightDir);
            if ~ok, apexH = apexRVO; end
            c.apex = apexH;
            c.leftDir = leftDir;
            c.rightDir = rightDir;
        end
end
end

function [vBest, vioCountBest] = chooseVelocity(vPref, c, vMax)
angles = linspace(0, 2*pi, 240+1); angles(end) = [];
speeds = linspace(0, vMax, 24);
cands = vPref;
for s = speeds
    ring = [s*cos(angles(:)), s*sin(angles(:))];
    cands = [cands; ring]; %#ok<AGROW>
end

bestCost = inf;
vBest = [0, 0];
vioCountBest = inf;
penalty = 100;
for i = 1:size(cands,1)
    v = cands(i,:);
    vio = isInsideCone(v, c);
    cost = norm(v - vPref) + penalty*double(vio);
    if cost < bestCost
        bestCost = cost;
        vBest = v;
        vioCountBest = double(vio);
    end
end
end

function drawConePatch(ax, c, len, colorRGB, labelText)
apex = c.apex;
p1 = apex + len * c.rightDir;
p2 = apex + len * c.leftDir;
patch(ax, [apex(1), p1(1), p2(1)], [apex(2), p1(2), p2(2)], colorRGB, ...
    'FaceAlpha', 0.18, 'EdgeColor', colorRGB, 'LineWidth', 1.3, ...
    'DisplayName', labelText);
line(ax, [apex(1), p1(1)], [apex(2), p1(2)], 'Color', colorRGB, 'LineWidth', 1.3);
line(ax, [apex(1), p2(1)], [apex(2), p2(2)], 'Color', colorRGB, 'LineWidth', 1.3);
plot(ax, apex(1), apex(2), '.', 'Color', colorRGB, 'MarkerSize', 16, 'DisplayName', [labelText, ' apex']);
end

function inside = isInsideCone(v, c)
w = v - c.apex;
inside = (cross2d(c.rightDir, w) >= 0) && (cross2d(w, c.leftDir) >= 0);
end

function [ok, p] = lineIntersection(p1, d1, p2, d2)
den = cross2d(d1, d2);
if abs(den) < 1e-11
    ok = false;
    p = [NaN, NaN];
    return;
end
t = cross2d((p2 - p1), d2) / den;
p = p1 + t * d1;
ok = true;
end

function z = cross2d(a, b)
z = a(1)*b(2) - a(2)*b(1);
end
