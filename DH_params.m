%params
theta=[20,45,45,80];
alpha=[90,0,0,0];
d=[5,0,0,0];
a=[0,5,5,5];

for i=1:length(theta)
    theta(i) = theta(i)*pi/180;
    alpha(i) = alpha(i)*pi/180;
end

i=1;
A12 = DHmatrix(theta(i), alpha(i), d(i), a(i));
i=2;
A23 = DHmatrix(theta(i), alpha(i), d(i), a(i));
i=3;
A34 = DHmatrix(theta(i), alpha(i), d(i), a(i));
i=4;
A45 = DHmatrix(theta(i), alpha(i), d(i), a(i));

A123=A12*A23;
A1234=A123*A34;
P=A12*A23*A34*A45;
X=[0,A12(1,4),A123(1,4),A1234(1,4),P(1,4)];
Y=[0,A12(2,4),A123(2,4),A1234(2,4),P(2,4)];
Z=[0,A12(3,4),A123(3,4),A1234(3,4),P(3,4)];


plot3(X,Y,Z);
axis([0 20 0 20 0 20])
grid on

function m = DHmatrix(theta,alpha,d,a)
    m = [cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta) a*cos(theta);sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta) a*sin(theta);0 sin(alpha) cos(alpha) d; 0 0 0 1];
end
