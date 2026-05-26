% This example demonstrates how you can use Path2D methods to compute path
% tracking errors for different kinds of error models.

clear
close all
clc

%% Problem setup
% Create a path
wpX = [0, 10, 15, 20, 25, 30];
wpY = [0,  0,  2,  2,  0, 0];
refPathP = PolygonPath.xy2Path(wpX, wpY);
refPathS = SplinePath.pp2Path(spline([0;refPathP.cumlengths()], [wpX; wpY]));

% Select one of the different path representations
refPath = refPathP;

% Vehicle position and lookahead length
xyVhcl = [6.2 -3];   
L = 6.0;


%% Pure Pursuit reference point
% Pure Pursuit tracks a reference path by choosing a lookahead point at
% distance L from the vehicle reference point. To find this point, we can
% use the method intersectCircle(), which finds the intersections of a
% circle at given orign and radius with the path.
P = exHlpErrMdlPurePursuit(refPath, xyVhcl, L);

figure(1)
refPath.plot();
hold on
plot(xyVhcl(1), xyVhcl(2), 'bo', 'MarkerFaceColor','b', 'DisplayName','Vehicle');
plot([P(1) xyVhcl(1)], [P(2) xyVhcl(2)], 'kp:', ...
    'MarkerFaceColor','r', ...
    'MarkerEdgeColor','r', ...
    'MarkerIndices',1, ...
    'DisplayName','Path tracking point');
title(sprintf('Pure Pursuit path tracking point (L=%.2f)', L));
hold off


%% Stanley reference point
% Stanley tracks a reference path by choosing the orthogonal projection
% (closest point) on the path. To find this point, we can use the method
% pointProjection().

P = exHlpErrMdlStanley(refPath, xyVhcl);

figure(2)
refPath.plot();
hold on
% plot(P(1), P(2), 'rp', 'MarkerFaceColor','r', 'DisplayName','Path tracking point');
plot(xyVhcl(1), xyVhcl(2), 'bo', 'MarkerFaceColor','b', 'DisplayName','Vehicle');
plot([P(1) xyVhcl(1)], [P(2) xyVhcl(2)], 'kp:', ...
    'MarkerFaceColor','r', ...
    'MarkerEdgeColor','r', ...
    'MarkerIndices',1, ...
    'DisplayName','Path tracking point');
title(sprintf('Stanley path tracking point'));
hold off
