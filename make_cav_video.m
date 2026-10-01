function make_cav_video(out, P, centerline, file)
%MAKE_CAV_VIDEO Chase-camera MP4 of one run, for the deliverable video.
%   MAKE_CAV_VIDEO(out, P, centerline, file) replays the logged ego and lead
%   motion in an Automated Driving Toolbox drivingScenario with three
%   lanes, the ego in the middle one, and writes file at 10 frames per
%   second with the speed, gap, ACC mode and lane offset overlaid.
%   VideoWriter cannot write MP4 on Linux, so frames go to a Motion JPEG
%   AVI first and ffmpeg converts it to H.264 MP4.

fps = 10;
scenario = drivingScenario('SampleTime', 1 / fps);
step = round(10 / centerline.ds);
centers = [centerline.x(1:step:end)' centerline.y(1:step:end)' zeros(numel(centerline.x(1:step:end)), 1)];
road(scenario, centers, 'Lanes', lanespec(3, 'Width', P.road.laneWidth));

% drivingScenario places a vehicle by its rear axle; the logs are at the CG
ego = vehicle(scenario, 'ClassID', 1, 'Length', P.ego.length, 'Width', P.ego.width, ...
    'RearOverhang', (P.ego.length - P.ego.a - P.ego.b) / 2, 'Wheelbase', P.ego.a + P.ego.b, ...
    'PlotColor', [0 0.45 0.74]);
lead = vehicle(scenario, 'ClassID', 1, 'Length', P.lead.length, 'Width', P.ego.width, ...
    'PlotColor', [0.85 0.33 0.1]);

fig = figure('Name', 'ACC and lane centering demo', 'Color', 'w', ...
    'Position', [100 100 1280 720]);
theme(fig, 'light');
ax = axes(fig);
chasePlot(ego, 'Parent', ax, 'ViewHeight', 4, 'ViewLocation', [-18 0], 'Meshes', 'on');
hud = annotation(fig, 'textbox', [0.01 0.80 0.42 0.19], 'FontName', 'monospaced', ...
    'FontSize', 13, 'BackgroundColor', [1 1 1 0.85], 'EdgeColor', 'none');

[folder, name] = fileparts(file);
aviFile = fullfile(folder, [name '.avi']);
writer = VideoWriter(aviFile, 'Motion JPEG AVI');
writer.FrameRate = fps;
writer.Quality = 95;
open(writer);

t = out.vEgo.Time;
following = acc_following(out, P);
frameTimes = 0:1 / fps:t(end);
for tf = frameTimes
    i = find(t >= tf, 1);
    psi = out.psi.Data(i);
    ego.Position = [out.X.Data(i) - P.ego.b * cos(psi), out.Y.Data(i) - P.ego.b * sin(psi), 0];
    ego.Yaw = rad2deg(psi);
    ego.Velocity = out.vEgo.Data(i) * [cos(psi) sin(psi) 0];

    sLead = out.sLead.Data(i);
    thLead = interp1(centerline.s, centerline.th, sLead, 'linear', 'extrap');
    lead.Position = [interp1(centerline.s, centerline.x, sLead, 'linear', 'extrap'), ...
        interp1(centerline.s, centerline.y, sLead, 'linear', 'extrap'), 0] ...
        - [cos(thLead) sin(thLead) 0] * P.lead.length * 0.3;
    lead.Yaw = rad2deg(thLead);

    updatePlots(scenario);
    modeNames = {'cruise at set speed', 'follow the lead'};
    hud.String = {
        sprintf('t = %4.1f s   ACC: %s', tf, modeNames{following(i) + 1})
        sprintf('ego  %5.1f m/s   lead %5.1f m/s', out.vEgo.Data(i), out.vLead.Data(i))
        sprintf('gap  %5.1f m     want %5.1f m', out.gap.Data(i), ...
            P.acc.d0 + P.acc.Th * out.vEgo.Data(i))
        sprintf('lane offset %+5.2f m   steer %+5.2f deg', out.ey.Data(i), ...
            rad2deg(out.steer.Data(i)))
        };
    drawnow;
    % The window manager can resize the figure; keep every frame the size
    % of the first
    img = frame2im(getframe(fig));
    if tf == 0
        frameSize = size(img, [1 2]);
    elseif ~isequal(size(img, [1 2]), frameSize)
        img = imresize(img, frameSize);
    end
    writeVideo(writer, img);
end
close(writer);

cmd = sprintf(['ffmpeg -y -loglevel error -i "%s" -c:v libx264 -pix_fmt yuv420p ' ...
    '-vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" "%s"'], aviFile, file);
[status, msg] = system(cmd);
if status ~= 0
    error('make_cav_video:ffmpeg', 'ffmpeg failed, AVI kept at %s:\n%s', aviFile, msg);
end
delete(aviFile);
end
