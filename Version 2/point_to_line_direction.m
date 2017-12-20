clear;
close;
clc;
figure(1)
x1=[0 0];
y1=[0 1];
plot(x1,y1)
x2=[0 0];
y2=[0 -1];
figure(1)
hold all
plot(x2,y2)
plot(x2(2),y2(2),'*')
xlim([-2 2])
ylim([-2 2])

result=x2(2)*y1(2)-x1(2)*y2(2);
if result>0
    disp('point 2 is on clockwise')
elseif result<0
    disp('point 2 is on counterclockwise')
else
    disp('point is on the line')
end