function My_createfigure(simout)
tic;
a=simout.Data;
x=a(:,1);
y=a(:,2);

axis([-100,100,-100,100]);
t=0:0.002:10;  
Nt=size(t,2);
xr=25*cos(t(1:Nt));
yr=35+25*sin(t(1:Nt));
plot(xr,yr,'g');
hold on;
for i=1:length(x)
    plot(xr,yr,'g');
    c=x(1:i,1);
    d=y(1:i,1); 
    plot(c,d,'r');

    hold on;
    pause(0.00001);
    axis([-40,40,0,80]);
    grid on;
end
toc

