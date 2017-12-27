function [X,V,A] = bezier(P,dt)
% First make sure that P is the correct size (4x2 or 2x4)
if ~isequal(size(P),[4 2]) && ~isequal(size(P),[2 4])
    disp("P is not the correct size. It should be 4x2 or 2x4.")
end
% Return the points of the curve in the same order that they came in
transpose = false;
if isequal(size(P),[4 2])
    transpose = true;
    P = P';
end
t = 0:dt:1;
% Position
x = (1 - t).^3 * P(1,1) + 3 * (1 - t).^2 .* t * P(1,2) + 3 * (1 - t) .* t.^2 * P(1,3) + t.^3 * P(1,4);
y = (1 - t).^3 * P(2,1) + 3 * (1 - t).^2 .* t * P(2,2) + 3 * (1 - t) .* t.^2 * P(2,3) + t.^3 * P(2,4);
X = [x;y];
% Velocity
x = 3 * (1 - t).^2 * (P(1,2) - P(1,1)) + 6 * (1 - t) .* t * (P(1,3) - P(1,2)) + 3 * t.^2 * (P(1,4) - P(1,3));
y = 3 * (1 - t).^2 * (P(2,2) - P(2,1)) + 6 * (1 - t) .* t * (P(2,3) - P(2,2)) + 3 * t.^2 * (P(2,4) - P(2,3));
V = [x;y];
% Acceleration
x = 6 * (1 - t) * (P(1,3) - 2 * P(1,2) + P(1,1)) + 6 * t * (P(1,4) - 2 * P(1,3) + P(1,2));
y = 6 * (1 - t) * (P(2,3) - 2 * P(2,2) + P(2,1)) + 6 * t * (P(2,4) - 2 * P(2,3) + P(2,2));
A = [x;y];
if transpose
    X = X';
    V = V';
    A = A';
end
end