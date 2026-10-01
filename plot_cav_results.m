function fig = plot_cav_results(out, P, road)
%PLOT_CAV_RESULTS Speed, gap, lane offset and steering for one run.
%   fig = PLOT_CAV_RESULTS(out, P, road) draws the road with the ego path
%   and three time histories: speeds, gap against the desired gap, and
%   lateral offset against the lane line margin.

t = out.vEgo.Time;
v = out.vEgo.Data(:);
gapDes = P.acc.d0 + P.acc.Th * v;
margin = P.road.laneWidth / 2 - P.ego.width / 2;
following = acc_following(out, P);
edges = diff([false; following; false]);
startT = t(edges(1:end-1) == 1);
endT = t(edges(2:end) == -1);

fig = figure('Name', 'ACC and lane centering', 'Color', 'w', ...
    'Position', [100 100 1200 800]);
theme(fig, 'light');
tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile(tl, [3 1]);
plot(ax, road.x, road.y, 'Color', [0.6 0.6 0.6], 'LineWidth', 6);
hold(ax, 'on');
plot(ax, out.X.Data, out.Y.Data, 'b', 'LineWidth', 1);
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
title(ax, 'Road (gray) and ego path (blue)');

ax = nexttile(tl);
xregion(ax, startT, endT, 'FaceColor', [0 0.6 0], 'FaceAlpha', 0.1);
hold(ax, 'on');
plot(ax, t, out.vLead.Data, 'r', t, v, 'b', 'LineWidth', 1.2);
grid(ax, 'on');
ylabel(ax, 'Speed (m/s)');
legend(ax, 'Following the lead', 'Lead', 'Ego', 'Location', 'southeast');
title(ax, 'ACC: speed (shaded while following)');

ax = nexttile(tl);
plot(ax, t, out.gap.Data, 'b', t, gapDes, 'k--', 'LineWidth', 1.2);
grid(ax, 'on');
ylabel(ax, 'Gap (m)');
legend(ax, 'Gap', sprintf('Desired: %g m + %g s x speed', P.acc.d0, P.acc.Th), ...
    'Location', 'northeast');
title(ax, 'ACC: gap to the lead');

ax = nexttile(tl);
plot(ax, t, out.ey.Data, 'b', 'LineWidth', 1.2);
hold(ax, 'on');
yline(ax, [margin -margin], 'r--', 'Tire on lane line');
grid(ax, 'on');
ylabel(ax, 'Offset (m)');
xlabel(ax, 'Time (s)');
title(ax, 'Lane centering: offset from lane center');
legend(ax, 'off');

drawnow;
end
