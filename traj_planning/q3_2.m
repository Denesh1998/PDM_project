close all
[x,y] = meshgrid(0:1:20,10:1:100);
a = 0.5*(12-x)./abs(12-x);
b = -0.1*ones(size(y));

[x1,y1] = meshgrid(0:1:0,10:1:100);
a1 = 0*ones(size(x1));
b1 = 0.4*ones(size(y1));

[x2,y2] = meshgrid(12,10:1:100);
a2 = 0*ones(size(x2));
b2 = -0.1*ones(size(y2));



figure

quiver(x1,y1,a1,b1,0,'m','filled')
hold on
quiver(x2,y2,a2,b2,0,'k','filled')
hold on 
quiver(x,y,a,b,0,'k','filled')

xlabel("x_1 [m]")
ylabel("x_2 [%]")
title("Phase portrait of overall BT")
xlim([-2 22])
ylim([0 110])