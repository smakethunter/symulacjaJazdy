function [sub_x, sub_y] = OGCS2Subscripts(ogcs_x, ogcs_y, resolution)
%OGCS2SUBSCRIPTS convert continuous CS to discrete grid subscripts
sub_x = floor(ogcs_x/resolution) + 1; % +1 - Matlab indexing
sub_y = floor(ogcs_y/resolution) + 1; % +1 - Matlab indexing
end

