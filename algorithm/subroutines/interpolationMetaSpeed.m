function [metaSpeed] = interpolationMetaSpeed(x,y)
%interpolationMetaSpeed(v_m,x,y) interpolates the speed values from the precalculated speed array from ship transit model.
% 
% This array - v_m - of the mean speeds. bst is array of 1 or 0, 1 when ship is beset in ice. For those cases v_m has entry of 0. ram is array of integers, the number of rams for the case.
% 15 minute simulations.
% All arrays are indexed (i,j,k), where i is heq, j is hi and k is 200 cases of one instance of hi-heq combination.
% 
% In this function x corresponds to a column (ice thickness - hi), y to a row (equivalent ice thickness - heq).
% hi = 0.1:0.1:0.8; level ice thicknesses
% heq = 0.05:0.05:0.6; equivalent ice thicknesses

load environment/metaSpeed.mat
M = size(v_m,1);
N = size(v_m,2);
Speed = mean(v_m,3);
interpolatedSpeed = @(x,y) interp2(1:N,1:M,Speed,x,y,'spline',0.2);
%interpolatedB = @(x,y) griddata(1:N,1:M,A,x,y); % alternative
metaSpeed=interpolatedSpeed(x,y);

end