%script for defining dynamics for an IMU driven EKF for a diff drive ground vehicle
syms x y v hx hy a o Bx By Mgx Mgy dT real 
R90=[0 -1;1 0];
states=[x y v hx hy]';

%prediction equation
f=[x+dT*v*hx;y+dT*v*hy; v+dT*a; [hx;hy]+dT*R90*[hx;hy]*o];
%derivative for EKF
F=jacobian(f,states);
FNum=matlabFunction(F)

%distance measurement
hd=norm([Bx;By]-[x;y])^2;
hdNum=matlabFunction(hd)
Hd=jacobian(hd,states);
HdNum=matlabFunction(Hd)

%magnetometer 
hm=[[Mgx Mgy]*[hx;hy];[Mgx Mgy]*(R90*[hx;hy])];
Hm=jacobian(hm,states);
HmNum=matlabFunction(Hm)


