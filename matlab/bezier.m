function X = bezier(P,dt)
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
x = (1 - t).^3 * P(1,1) + 3 * (1 - t).^2 .* t * P(1,2) + 3 * (1 - t) .* t.^2 * P(1,3) + t.^3 * P(1,4);
y = (1 - t).^3 * P(2,1) + 3 * (1 - t).^2 .* t * P(2,2) + 3 * (1 - t) .* t.^2 * P(2,3) + t.^3 * P(2,4);
X = [x;y];
if transpose
    X = X';
end
end