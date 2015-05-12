function [ranges] = contiguousRange(v)

v = unique(v(:));
vdiff = [0; v(2:end)-v(1:end-1)];
idx = find(vdiff ~= 1);
ranges = [v(idx), [v(idx(2:end)-1); nan]];
ranges(ranges(:, 1) == ranges(:, 2), 2) = nan;

end

