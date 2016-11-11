function [distanceFromLine] = distance2curve2(waypoints,x2, y2)
% DISTANCE2CURVE2 calculates the shortest distance between a given point 
% x2,y2 and a line segment defined by the IB waypoints.
% First it calculates distances between a given point and all line
% segments.
% Second a shortest distance among all distances is taken and considered as
% a distanceFromLine.

x = [x2,y2]; %some point
i=[];
s=size(waypoints);
d=zeros(6,1);
for i=1:1:s(1,1)-1;
    a = [waypoints(i,1),waypoints(i,2)]; %segment points a,b
    b = [waypoints(i+1,1),waypoints(i+1,2)];
    
    d_ab = norm(a-b);
    d_ax = norm(a-x);
    d_bx = norm(b-x);

    if dot(a-b,x-b)*dot(b-a,x-a)>=0
        A = [a,1;b,1;x,1];
        dist(i) = abs(det(A))/d_ab;        
    else
        dist(i) = min(d_ax, d_bx);
    end
    distanceFromLine=min(dist);
end % end for

end

