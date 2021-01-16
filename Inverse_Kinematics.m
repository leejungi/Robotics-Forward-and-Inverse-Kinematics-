%DH params
theta=[0,0,0,0];
alpha=[90,0,0,0];
d=[5,0,0,0];
a=[0,5,5,5];

%Inverse Kinematics
%Target coordinate
Px = 10;
Py = 8;
Pz = 0;

%Assumption for easy calculation
%Ignore first joint, setting joint2 as 0,0
%Coordinate of joint 4
Px4= sqrt(Px^2+Py^2); 
Py4= Pz-d(1);

%Theta 2+3+4 cos and sin value
ct = Px4/sqrt(Px4^2+Py4^2);
st = Py4/sqrt(Px4^2+Py4^2);

%Joint 3 coordinate
Px3 = Px4-a(4)*ct;
Py3 = Py4-a(4)*st;

%Theta 3 cos and sin value
c3 = (Px3^2 + Py3^2 -a(2)^2-a(3)^2)/(2*a(2)*a(3));
s3 = -sqrt(1-c3^2); %+-

theta(3) = atan2(s3,c3);

%Theta 2 cos and sin value
c2=((a(2)+a(3)*c3)*Px3+a(3)*s3*Py3)/(Px3^2+Py3^2);
s2=((a(2)+a(3)*c3)*Py3-a(3)*s3*Px3)/(Px3^2+Py3^2);

theta(2) = atan2(s2,c2);

theta(4) = atan2(st,ct) - theta(3) - theta(2);

theta(1) = atan(Py/Px);


%Forward Kinematics
theta_degree = [0,0,0,0];
for i=1:length(theta)
%    theta(i) = theta(i)*pi/180;
    theta_degree(i) = theta(i)*180/pi;
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

A= [0,0,0,0];
A(1) = sqrt((A12(1,4))^2+(A12(2,4))^2+(A12(3,4))^2);
A(2) = sqrt((A12(1,4)-A123(1,4))^2+(A12(2,4)-A123(2,4))^2+(A12(3,4)-A123(3,4))^2);
A(3) = sqrt((A123(1,4)-A1234(1,4))^2+(A123(2,4)-A1234(2,4))^2+(A123(3,4)-A1234(3,4))^2);
A(4) = sqrt((A1234(1,4)-P(1,4))^2+(A1234(2,4)-P(2,4))^2+(A1234(3,4)-P(3,4))^2);

fprintf("Link Length Check\n");
fprintf("Origin: [%d %d %d %d] Check: [%d %d %d %d]\n",d(1),a(2),a(3),a(4),A(1),A(2),A(3),A(4));
fprintf("End Effector Pointing: [%d, %d, %d]\n", P(1,4),P(2,4),P(3,4));

function m = DHmatrix(theta,alpha,d,a)
    m = [cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta) a*cos(theta);sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta) a*sin(theta);0 sin(alpha) cos(alpha) d; 0 0 0 1];
end