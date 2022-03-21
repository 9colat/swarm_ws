%simulating diff drive ground vehicle
%states: [position (1X2), forward vel (1X1), heading (1X2)]

Beacons=[[0;0] [-5;5] [5;5]];
R=1e-4; %measurement covariance
P=eye(5); %initial estimate covariance

%simulation time
T=100;
dT=0.1;
N=ceil(T/dT);
states=zeros(5,N);
e_states=states;
states(:,1)=[0 0 1 1 0]';
R90=[0 -1;1 0]
angle_small=0.1;
Rsmall=[cos(angle_small) -sin(angle_small);sin(angle_small) cos(angle_small)];
Hstart=Rsmall*[1;0];
e_states(:,1)=[0.1 0 1 Hstart']';
omega=0.2;

for i=1:N-1
    states(1:2,i+1)=states(1:2,i)+dT*states(4:5,i)*states(3,i); %position update
    states(3,i+1)=states(3,i)+0;  %0 for acc
    states(4:5,i+1)=states(4:5,i)+dT*R90*states(4:5,i)*omega;
    
    %prediction
    e_states(1:2,i+1)=e_states(1:2,i)+dT*e_states(4:5,i)*e_states(3,i); %position update
    e_states(3,i+1)=e_states(3,i)+0;  %0 for acc
    e_states(4:5,i+1)=e_states(4:5,i)+dT*R90*e_states(4:5,i)*omega;
    
    %measurement
    B_index=rem(i,3)+1;
    Beacon=Beacons(:,B_index);
    y=norm(states(1:2,i)-Beacon)^2+randn(1)*0.1;
    ye=norm(e_states(1:2,i+1)-Beacon)^2;
    yd=y-ye;
    H=HdNum(Beacon(1),Beacon(2),e_states(1,i),e_states(2,i));
    S=H*P*H'+R;
    K=P*H'*inv(S);
    e_states(:,i+1)=e_states(:,i+1)+K*yd;
    P=(eye(5)-K*H)*P; 
end
plot(states(1,:),states(2,:),'.',Beacons(1,:),Beacons(2,:),'*',e_states(1,:),e_states(2,:),'r.')
plot(states(4,:),states(5,:),'.',e_states(4,:),e_states(5,:),'r.')

