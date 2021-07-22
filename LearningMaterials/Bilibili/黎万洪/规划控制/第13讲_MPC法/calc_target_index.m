function [target_idx, error] = calc_target_index(x,y, refPos_x,refPos_y)

i = 1:length(refPos_x)-1;
dist = sqrt((refPos_x(i)-x).^2 + (refPos_y(i)-y).^2);
[value, target_idx] = min(dist);

if y<refPos_y(target_idx)
    error = -value;
else
    error = value;
end
end