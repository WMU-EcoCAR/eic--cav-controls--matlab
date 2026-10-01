function fig = plot_cav_compare(outs, names, P)
%PLOT_CAV_COMPARE Overlay the runs of several controllers on one figure.
%   fig = PLOT_CAV_COMPARE(outs, names, P) plots ego speed, gap, lateral
%   offset and steering angle for each run in the cell array outs.

colors = [0 0.45 0.74; 0.85 0.33 0.1; 0.47 0.67 0.19];
fig = figure('Name', 'Controller comparison', 'Color', 'w', ...
    'Position', [100 100 1000 800]);
theme(fig, 'light');
tl = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
margin = P.road.laneWidth / 2 - P.ego.width / 2;

axSpeed = nexttile(tl);
plot(axSpeed, outs{1}.vLead.Time, outs{1}.vLead.Data, 'k:', 'LineWidth', 1.2, ...
    'DisplayName', 'Lead');
axGap = nexttile(tl);
axOffset = nexttile(tl);
yline(axOffset, [margin -margin], 'r--', 'HandleVisibility', 'off');
axSteer = nexttile(tl);
hold([axSpeed axGap axOffset axSteer], 'on');

for i = 1:numel(outs)
    o = outs{i};
    t = o.vEgo.Time;
    c = colors(i, :);
    plot(axSpeed, t, o.vEgo.Data, 'Color', c, 'LineWidth', 1.2, 'DisplayName', names{i});
    plot(axGap, t, o.gap.Data ./ max(o.vEgo.Data, 1), 'Color', c, 'LineWidth', 1.2);
    plot(axOffset, t, o.ey.Data, 'Color', c, 'LineWidth', 1.2);
    plot(axSteer, t, rad2deg(o.steer.Data), 'Color', c, 'LineWidth', 1.2);
end
yline(axGap, P.acc.Th, 'k--', sprintf('PID target %g s', P.acc.Th));
ylim(axGap, [0 5]);

ylabel(axSpeed, 'Speed (m/s)');
ylabel(axGap, 'Time gap (s)');
ylabel(axOffset, 'Lane offset (m)');
ylabel(axSteer, 'Steer (deg)');
xlabel(axSteer, 'Time (s)');
legend(axSpeed, 'Location', 'southeast');
title(axSpeed, 'ACC: ego speed');
title(axGap, 'ACC: time gap to the lead (gap / speed)');
title(axOffset, 'Lane centering: offset from lane center (red: tire on lane line)');
title(axSteer, 'Lane centering: road wheel angle');
grid([axSpeed axGap axOffset axSteer], 'on');
drawnow;
end
