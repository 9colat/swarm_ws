ys=SLAMGOTlog(:,4);
N=length(ys);
%R=400:800; %range
R=102:N; %range
xg=SLAMGOTlog(R,1);
yg=SLAMGOTlog(R,2);
xs=SLAMGOTlog(R,3);
ys=SLAMGOTlog(R,4);
N=length(ys);

A=[xs'*xs xs'*ys sum(xs)
    xs'*ys ys'*ys sum(ys)
    sum(xs) sum(ys) N];
B1=[xg'*xs;xg'*ys;sum(xg)];
B2=[yg'*xs;yg'*ys;sum(yg)];

X1=A\B1;
X2=A\B2;

AT=[X1(1:2)';X2(1:2)']
BT=[X1(3);X2(3)];

Xsg=AT*[xs';ys']+BT;

%plot(xg,yg,'*',Xsg(1,:),Xsg(2,:),'.')
%axis([2e4 2.5e4 0 6000])