%RUN_CAV_DEMO Simulate ACC and lane centering with PID and with MPC.
%   Builds cav_pid.slx and cav_mpc.slx if they are missing, runs both on
%   the same scenario, prints the pass/fail checks, and saves the plots in
%   results/. Run makeVideo = true; run_cav_demo for a chase-camera MP4 of
%   each run as well.

makeVideo = exist('makeVideo', 'var') && makeVideo;

here = fileparts(mfilename('fullpath'));
P = cav_params();
road = make_road(P.road.segments, P.road.ds, P.road.transition);
resultsDir = fullfile(here, 'results');
if ~isfolder(resultsDir)
    mkdir(resultsDir);
end

controllers = {'pid', 'mpc'};
outs = cell(size(controllers));
for i = 1:numel(controllers)
    mdl = ['cav_' controllers{i}];
    if ~isfile(fullfile(here, [mdl '.slx']))
        build_cav_model(controllers{i});
    end
    fprintf('--- %s\n', upper(controllers{i}));
    outs{i} = sim(mdl);
    cav_metrics(outs{i}, P, true);
    fig = plot_cav_results(outs{i}, P, road);
    exportgraphics(fig, fullfile(resultsDir, [controllers{i} '_results.png']), 'Resolution', 150);
    if makeVideo
        make_cav_video(outs{i}, P, road, fullfile(resultsDir, [controllers{i} '_demo.mp4']));
    end
end

fig = plot_cav_compare(outs, upper(controllers), P);
exportgraphics(fig, fullfile(resultsDir, 'pid_vs_mpc.png'), 'Resolution', 150);
