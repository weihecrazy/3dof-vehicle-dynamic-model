% 3DOF model vehicle parameters
%车身参数参考同CarSim-high CG SUV
g=9.81;
m=1862;%1942;
u=22;
ms=1592;
a=1.18;
b=1.77;
h=0.4;
Ix=614;
Iz=2488;
kf=74788;
kr=70741;
k1=57697;
c1=3996;
a11=(-2*kf-2*kr)*Ix/((m*Ix-ms*ms*h*h)*u);
a12=(((-2*a*kf+2*b*kr)*Ix)/(m*Ix-ms*ms*h*h))-u;
a13=ms*h*c1/(ms*ms*h*h-m*Ix);
a14=ms*h*(ms*h*g-k1)/(m*Ix-ms*ms*h*h);

a21=(-2*kf*a+2*kr*b)/(Iz*u);
a22=(-2*a*a*kf-2*b*b*kr)/(Iz*u);

a31=ms*h*(2*kf+2*kr)/((ms*ms*h*h-m*Ix)*u);
a32=ms*h*(2*a*kf-2*b*kr)/((ms*ms*h*h-m*Ix)*u);
a33=m*c1/(ms*ms*h*h-m*Ix);
a34=m*(k1-ms*g*h)/(ms*ms*h*h-m*Ix);

b1=2*kf*Ix/(m*Ix-ms*ms*h*h);
b2=2*kf*a/Iz;
b3=-2*kf*ms*h/(ms*ms*h*h-m*Ix);


A=[a11 a12 a13 a14;a21 a22 0 0;a31 a32 a33 a34;0 0 1 0];
B=[b1;b2;b3;0];
C=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
D=0;

t=[0:0.01:6];
U=0.5*pi/180*sin(1/3*2*pi*t);

Y=lsim(A,B,C,D,U,t);	



% subplot, subplot(221)
% plot(t,Y(:,1),'r'); grid
% xlabel('time (sec)')
% ylabel('Lateral speed (m/sec)')
% 
% subplot(222)
% plot(t,Y(:,2)*180/pi,'r'); grid
% xlabel('time (sec)')
% ylabel('Yaw rate (deg/sec)')
% 
% subplot(223)
% plot(t,Y(:,3),'r'); grid
% xlabel('time (sec)')
% ylabel('Lat. Accel.(m/sec^2)')
% 
% subplot(224)
% plot(t,U*180/pi,'r'); grid
% xlabel('time (sec)')
% ylabel('Steering (deg)')
