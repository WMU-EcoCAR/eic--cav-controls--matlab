function road = make_road(segments, ds, transition)
%MAKE_ROAD Lane centerline from straight and constant-radius segments.
%   road = MAKE_ROAD(segments, ds, transition) chains the segments, each a
%   row of [length radius] with radius 0 for a straight and positive radius
%   for a left turn, starting at the origin heading along +X. Curvature
%   ramps linearly over transition meters at each change, as a clothoid
%   would on a real road, keeping each curve's total turn. It returns
%   points spaced ds apart with arc length s, position x and y, heading th
%   and curvature k.

if nargin < 3
    transition = 0;
end
k = [];
for i = 1:size(segments, 1)
    radius = segments(i, 2);
    kappa = 0;
    if radius ~= 0
        kappa = 1 / radius;
    end
    k = [k, kappa * ones(1, round(segments(i, 1) / ds))]; %#ok<AGROW>
end
if transition > 0
    k = movmean(k, round(transition / ds));
end

n = numel(k) + 1;
s = (0:n - 1) * ds;
th = [0, cumsum(k) * ds];
thMid = th(1:end - 1) + k * ds / 2;
x = [0, cumsum(ds * cos(thMid))];
y = [0, cumsum(ds * sin(thMid))];

road.s = s;
road.x = x;
road.y = y;
road.th = th;
road.k = [k, k(end)];
road.ds = ds;
end
