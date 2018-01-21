xPre = rand(1,2);
thetaPre = 2*pi*rand-pi;
vPre = 2*rand;

xTarg = rand(1,2);
thetaTarg = 2*pi*rand-pi;
vTarg = 2*rand;

xPre = [0 0];
thetaPre = 0;
vPre = 2;
xTarg = [1 2];
thetaTarg = pi/2;
vTarg = 2;

dt = 0.05;
t = 0:dt:1;

[X,V,A,P] = connect_waypoints(xPre,thetaPre,vPre,xTarg,thetaTarg,vTarg,dt);

vTot = sqrt(V(:,1).^2 + V(:,2).^2);
theta = atan2(V(:,2),V(:,1));
kappa = (V(:,1).*A(:,2) - V(:,2).*A(:,1))./((V(:,1).^2 + V(:,2).^2).^(3/2));
omega = vTot.*kappa;

% 
%            x'y" - y'x"
% κ(t)  = --------------------
%          (x'² + y'²)^(3/2)
%

assert(vTot(1) - vPre < 0.00000001)
assert(vTot(end) - vTarg < 0.00000001)

figure(1)
plot(X(:,1),X(:,2),'.r')
hold on
plot(P(:,1),P(:,2),'ok')
xlim([-2 3])
ylim([-2 3])
axis square
hold off

figure(2)
subplot(3,1,1)
plot(t,X(:,1),t,X(:,2))
ylabel("x,y")
xlabel("t")
subplot(3,1,2)
plot(t,V(:,1),t,V(:,2))
ylabel("v_x,v_y")
xlabel("t")
subplot(3,1,3)
plot(t,A(:,1),t,A(:,2))
ylabel("a_x,a_y")
xlabel("t")

figure(3)
subplot(4,1,1)
plot(t,X(:,1),t,X(:,2))
ylabel("x,y")
xlabel("t")
subplot(4,1,2)
plot(t,vTot)
hold on
plot(0,vPre,'+r',1,vTarg,'+r')
hold off
ylabel("v_{tot}")
xlabel("t")
subplot(4,1,3)
plot(t,theta)
hold on
plot(0,thetaPre,'+r',1,thetaTarg,'+r')
hold off
ylabel("\theta")
xlabel("t")
ylim([-pi pi])
subplot(4,1,4)
plot(t,omega)
hold on
omegaPre = 0;
omegaTarg = 0;
plot(0,omegaPre,'+r',1,omegaTarg,'+r')
hold off
ylabel("\omega")
xlabel("t")