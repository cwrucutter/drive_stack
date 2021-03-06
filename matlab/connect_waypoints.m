function [X,V,A,P] = connect_waypoints(xPre,thetaPre,vPre,xTarg,thetaTarg,vTarg,dt)
% First make sure all the inputs are the correct size
if ~isequal(size(xPre),[2 1]) && ~isequal(size(xPre),[1 2])
    disp("xPre is not the correct size. It should be 2x1 or 1x2.")
end
if ~isequal(size(xTarg),[2 1]) && ~isequal(size(xTarg),[1 2])
    disp("xTarg is not the correct size. It should be 2x1 or 1x2.")
end
if ~isequal(size(xPre),size(xTarg))
    disp("xTarg and xPre are not the same dimensions.")
end
if ~isscalar(thetaPre)
    disp("thetaPre is not a scalar.")
end
if ~isscalar(vPre)
    disp("vPre is not a scalar.")
end
if ~isscalar(thetaTarg)
    disp("thetaPre is not a scalar.")
end
if ~isscalar(vTarg)
    disp("vPre is not a scalar.")
end
% Check the rotation of xPre and xTarg
transpose = false;
if isequal(size(xPre),[2 1])
    transpose = true;
    xPre = xPre';
    xTarg = xTarg';
end

% Divide by three to make the velocity calculated below the same as vPre
% and vTarg. Not sure why this is. But, it works!
x1 = [xPre(1) + vPre*cos(thetaPre)/3 xPre(2) + vPre*sin(thetaPre)/3];
x2 = [xTarg(1) - vTarg*cos(thetaTarg)/3 xTarg(2) - vTarg*sin(thetaTarg)/3];

P = [xPre;x1;x2;xTarg];

% Rotate P (and thus X) if xPre and xTarg were rotated.
if transpose
    P = P';
end

[X,V,A] = bezier(P,dt);
end