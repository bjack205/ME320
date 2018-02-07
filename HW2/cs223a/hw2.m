T = linspace(0,10,10000);
for i = 1:length(T)
t = T(i);
x(1:3,i) = [-sin(t).*cos(t) -cos(t).*sin(t); cos(t).*cos(t) -sin(t).*sin(t); 0 cos(t)]*[1;1];
end
figure
plot3(x(1,:),x(2,:),x(3,:))
xlabel('x')
ylabel('y')
zlabel('z')
title('Trajectory')
