Mag=rotmat(3*pi/4)*[1 0]'

h1=rotmat(45*pi/180)*[1 0]';
h2=rotmat(0.1)*h1;

Meas1=[Mag'*h1;Mag'*(rotmat(pi/2)*h1)]
Meas2=[Mag'*h2;Mag'*(rotmat(pi/2)*h2)]

[Mag';-(rotmat(pi/2)*Mag)']*(h1-h2)
dM=Meas1-Meas2


function R=rotmat(angle)
R=[cos(angle) -sin(angle);sin(angle) cos(angle)];
end