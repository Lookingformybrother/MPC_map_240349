function [r,c]=get_best_open_node(openSet,fScore)
tmp = fScore;
tmp(~openSet) = inf;
[~, idx] = min(tmp(:));
[r,c] = ind2sub(size(tmp),idx);
end
