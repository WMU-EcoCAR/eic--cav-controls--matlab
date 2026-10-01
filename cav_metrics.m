function M = cav_metrics(out, P, verbose)
%CAV_METRICS Score one run: does the car stay in its lane and avoid the lead?
%   M = CAV_METRICS(out, P) returns lane-keeping and following measures
%   from the simulation output. CAV_METRICS(out, P, true) also prints them.

if nargin < 3
    verbose = false;
end
t = out.vEgo.Time;
v = out.vEgo.Data(:);
gap = out.gap.Data(:);
ey = out.ey.Data(:);
steer = out.steer.Data(:);
following = acc_following(out, P);

laneMargin = P.road.laneWidth / 2 - P.ego.width / 2;
M.maxLateral = max(abs(ey));
M.rmsLateral = rms(ey);
M.inLane = M.maxLateral < laneMargin;
M.laneMargin = laneMargin;
M.maxSteerDeg = rad2deg(max(abs(steer)));
M.minGap = min(gap);
M.collision = M.minGap <= 0;
M.minTimeGap = min(gap(v > 1) ./ v(v > 1));
M.maxDecel = -min(gradient(v, t));
M.maxSpeed = max(v);
M.timeFollowing = sum(following) * mean(diff(t));
gapDes = P.acc.d0 + P.acc.Th * v;
settled = following & t > t(end) - 10;
M.finalGapError = mean(gap(settled) - gapDes(settled));
if isempty(M.finalGapError) || isnan(M.finalGapError)
    M.finalGapError = NaN;
end

if verbose
    fprintf('Lane centering: max |offset| %.2f m, RMS %.2f m, margin to lane line %.2f m -> %s\n', ...
        M.maxLateral, M.rmsLateral, laneMargin, passFail(M.inLane));
    fprintf('                max steer %.1f deg\n', M.maxSteerDeg);
    fprintf('ACC:            min gap %.1f m, min time gap %.2f s -> %s\n', ...
        M.minGap, M.minTimeGap, passFail(~M.collision));
    fprintf('                max decel %.2f m/s^2, max speed %.2f m/s (set %.1f)\n', ...
        M.maxDecel, M.maxSpeed, P.acc.vSet);
    fprintf('                following for %.1f s, gap error over the last 10 s %.1f m\n', ...
        M.timeFollowing, M.finalGapError);
end
end

function s = passFail(ok)
if ok
    s = 'PASS';
else
    s = 'FAIL';
end
end
