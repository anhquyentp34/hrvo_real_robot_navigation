% HRVO demo based on:
% Snape et al., "The Hybrid Reciprocal Velocity Obstacle", IEEE T-RO 2011.
%
% This script compares VO, RVO, and HRVO in a circle-crossing scenario.
% Run:
%   hrvo_demo
%
% You can tune parameters in createDefaultParams().

clear; clc; close all;

params = createDefaultParams();
methods = ["VO", "RVO", "HRVO"];

for m = 1:numel(methods)
    simulateScenario(methods(m), params);
end

function params = createDefaultParams()
params.nAgents = 16;
params.worldRadius = 6.0;
params.agentRadius = 0.25;
params.vMax = 1.2;
params.dt = 0.08;
params.steps = 350;

% Velocity sampling for argmin ||v - v_pref|| outside constraints.
params.nAngleSamples = 72;
params.nSpeedSamples = 8;

% Penalty used when all candidates violate constraints.
params.violationPenalty = 50;

% Visualization
params.drawEvery = 2;
end

function simulateScenario(method, params)
N = params.nAgents;
R = params.worldRadius;
r = params.agentRadius;
dt = params.dt;

angles = linspace(0, 2*pi, N+1);
angles(end) = [];
pos = [R*cos(angles(:)), R*sin(angles(:))];
goal = -pos; % antipodal goals
vel = zeros(N, 2);
traj = zeros(N, 2, params.steps);

figure('Color', 'w', 'Name', sprintf('%s demo', method));
ax = axes; hold(ax, 'on'); axis(ax, 'equal');
xlim(ax, [-R-1.5, R+1.5]); ylim(ax, [-R-1.5, R+1.5]);
title(ax, sprintf('%s: circle crossing', method), 'FontWeight', 'bold');
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');
grid(ax, 'on');

for k = 1:params.steps
    newVel = zeros(N, 2);

    for i = 1:N
        toGoal = goal(i,:) - pos(i,:);
        dGoal = norm(toGoal);
        if dGoal < 1e-3
            vPref = [0, 0];
        else
            vPref = params.vMax * toGoal / dGoal;
        end

        constraints = [];
        for j = 1:N
            if j == i
                continue;
            end
            c = buildConstraint(pos(i,:), vel(i,:), pos(j,:), vel(j,:), 2*r, method);
            if ~isempty(c)
                constraints = [constraints; c]; %#ok<AGROW>
            end
        end

        newVel(i,:) = selectVelocity(vPref, constraints, params);
    end

    vel = newVel;
    pos = pos + dt * vel;
    traj(:,:,k) = pos;

    if mod(k, params.drawEvery) == 0 || k == 1 || k == params.steps
        cla(ax);
        drawScene(ax, pos, goal, traj(:,:,1:k), r, method, k, params.steps);
        drawnow;
    end
end
end

function c = buildConstraint(pA, vA, pB, vB, combinedRadius, method)
% Build one forbidden cone in velocity space for agent A caused by B.
% Cone is represented by apex and two edge directions (left/right).

relPos = pB - pA;
dist = norm(relPos);
if dist < 1e-6
    c = [];
    return;
end

if dist <= combinedRadius
    % Overlap case: force agent to move away from the neighbor immediately.
    away = -relPos / dist;
    c.apex = [0, 0];
    c.leftDir = rot2d(away, deg2rad(20));
    c.rightDir = rot2d(away, -deg2rad(20));
    c.kind = "emergency";
    return;
end

theta = atan2(relPos(2), relPos(1));
alpha = asin(min(0.999, combinedRadius / dist));

leftDir = [cos(theta + alpha), sin(theta + alpha)];
rightDir = [cos(theta - alpha), sin(theta - alpha)];

apexVO = vB;
apexRVO = 0.5 * (vA + vB);

switch upper(method)
    case "VO"
        c.apex = apexVO;
        c.leftDir = leftDir;
        c.rightDir = rightDir;
        c.kind = "VO";
    case "RVO"
        c.apex = apexRVO;
        c.leftDir = leftDir;
        c.rightDir = rightDir;
        c.kind = "RVO";
    otherwise
        % HRVO:
        % Decide side of current velocity w.r.t. centerline of RVO.
        centerDir = relPos / dist;
        side = cross2d(centerDir, vA - apexRVO);

        if side >= 0
            % vA is on left of centerline (CCW convention):
            % keep left edge from RVO, replace right edge by VO edge.
            [ok, apexH] = lineIntersection(apexRVO, leftDir, apexVO, rightDir);
            if ~ok
                apexH = apexRVO;
            end
            c.apex = apexH;
            c.leftDir = leftDir;
            c.rightDir = rightDir;
        else
            % vA is on right of centerline:
            % keep right edge from RVO, replace left edge by VO edge.
            [ok, apexH] = lineIntersection(apexVO, leftDir, apexRVO, rightDir);
            if ~ok
                apexH = apexRVO;
            end
            c.apex = apexH;
            c.leftDir = leftDir;
            c.rightDir = rightDir;
        end
        c.kind = "HRVO";
end
end

function vBest = selectVelocity(vPref, constraints, params)
% Discrete search in velocity space:
% minimize ||v - v_pref||, while preferring candidates outside all cones.

angles = linspace(0, 2*pi, params.nAngleSamples+1);
angles(end) = [];
speeds = linspace(0, params.vMax, params.nSpeedSamples);

% Include exact preferred velocity candidate as first choice.
candidates = vPref;
for s = speeds
    cs = [s*cos(angles(:)), s*sin(angles(:))];
    candidates = [candidates; cs]; %#ok<AGROW>
end

bestCost = inf;
vBest = [0, 0];

for idx = 1:size(candidates, 1)
    v = candidates(idx,:);
    vio = countViolations(v, constraints);
    cost = norm(v - vPref) + params.violationPenalty * vio;
    if cost < bestCost
        bestCost = cost;
        vBest = v;
    end
end
end

function n = countViolations(v, constraints)
n = 0;
for i = 1:numel(constraints)
    if isInsideCone(v, constraints(i))
        n = n + 1;
    end
end
end

function inside = isInsideCone(v, c)
w = v - c.apex;
if norm(w) < 1e-12
    inside = true;
    return;
end

% Cone between rightDir and leftDir (counter-clockwise).
inside = (cross2d(c.rightDir, w) >= 0) && (cross2d(w, c.leftDir) >= 0);
end

function drawScene(ax, pos, goal, traj, radius, method, step, totalSteps)
N = size(pos,1);

for i = 1:N
    % Keep trajectory as Kx2 robustly for all K (including K=1).
    tr = reshape(traj(i,1:2,:), 2, [])';
    tr = tr(~any(tr == 0,2), :);
    if ~isempty(tr)
        plot(ax, tr(:,1), tr(:,2), 'LineWidth', 1.2);
    end
end

for i = 1:N
    plot(ax, goal(i,1), goal(i,2), 'kx', 'MarkerSize', 8, 'LineWidth', 1.2);
    viscircles(ax, pos(i,:), radius, 'Color', 'k', 'LineWidth', 0.8);
    text(pos(i,1), pos(i,2), sprintf('%d', i), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontSize', 8, 'FontWeight', 'bold');
end

title(ax, sprintf('%s | step %d/%d', method, step, totalSteps), 'FontWeight', 'bold');
end

function [ok, p] = lineIntersection(p1, d1, p2, d2)
% Solve p1 + t*d1 = p2 + s*d2.
den = cross2d(d1, d2);
if abs(den) < 1e-10
    ok = false;
    p = [NaN, NaN];
    return;
end
t = cross2d((p2 - p1), d2) / den;
p = p1 + t*d1;
ok = true;
end

function vOut = rot2d(vIn, ang)
c = cos(ang); s = sin(ang);
R = [c -s; s c];
vOut = (R * vIn(:))';
end

function z = cross2d(a, b)
z = a(1)*b(2) - a(2)*b(1);
end
