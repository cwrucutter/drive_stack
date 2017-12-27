xPre = rand(1,2);
thetaPre = 2*pi*rand;
vPre = 2*rand;

xTarg = rand(1,2);
thetaTarg = 2*pi*rand;
vTarg = 2*rand;

dt = 0.05;

[X, P] = connect_waypoints(xPre,thetaPre,vPre,xTarg,thetaTarg,vTarg,dt);

figure(1)
plot(X(:,1),X(:,2),'.r')
hold on
plot(P(:,1),P(:,2),'ok')
xlim([-2 3])
ylim([-2 3])
axis square
hold off