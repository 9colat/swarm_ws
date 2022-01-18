


plot(linspace(0,length(A),length(A)),abs(A),'r');
hold on

plot(linspace(0,length(B),length(B)),abs(B/2*pi),'b');
plot(linspace(0,length(C),length(C)),abs(C/2*pi),'r');
legend("Right","Left")
