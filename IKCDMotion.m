%%%%%%%%%%%%% Raphael Nagel %%%%%%%%%%%
%%%%%%%%%%% Co-Design       %%%%%%%%%
%%%%%%%%%%%%%% December 2012 %%%%%%%%%%%%%
% Forward ands inverse kinematics

clear all
close all
clc

deg2rad = pi/180;
rad2deg = 180/pi;

%% Links Length
l1 = 8.5+9;
l2 = 10 ;
l3 = 12 ;
d5 = 4.5;


%% Desired Position input
x_in = 10;
y_in = 10;
z_in = 0;

%%

p0e = [x_in y_in z_in]';

%%print
disp('Desired position =')
disp(p0e)
if norm(p0e) > l2+l3+d5   %origin is l1 above the table...
    error('desired position is out of the workspace')
end

%% %%%%%%%%%%%% Inverse Kinematics of the Lynx Motion Arm 6 DOF manipulator %%%%%%%%%%%%%%%%%
%% Inverse Kinematic Postition
%theta 1(rotation around base)
q1  = atan2(p0e(2),p0e(1));

p0c = p0e - d5.*[cos(q1) sin(q1) 0]';


xc = p0c(1,1);
yc = p0c(2,1);
zc = p0c(3,1);

disp('Wrist Position =')
disp(p0c)

%%




%% Geometric solution of q1, q2 and q3

%set 1

q31 = atan2(+sqrt(1-((xc^2+yc^2+zc^2-l2^2-l3^2)/(2*l2*l3))^2),((xc^2+yc^2+zc^2-l2^2-l3^2)/(2*l2*l3)));
q21 = -atan2(((l3*sin(q31))/sqrt(xc^2+yc^2+zc^2)),+sqrt(1-((l3*sin(q31))/sqrt(xc^2+yc^2+zc^2))^2))+atan2(zc,sqrt(xc^2+yc^2));
q41 = pi/2 -(q21+q31);



%set 2
q32 = atan2(-sqrt(1-((xc^2+yc^2+zc^2-l2^2-l3^2)/(2*l2*l3))^2),((xc^2+yc^2+zc^2-l2^2-l3^2)/(2*l2*l3)));
q22 = -atan2(((l3*sin(q32))/sqrt(xc^2+yc^2+zc^2)),+sqrt(1-((l3*sin(q32))/sqrt(xc^2+yc^2+zc^2))^2))+atan2(zc,sqrt(xc^2+yc^2));

q42 = pi/2 -(q22+q32);

smax = 100000;
smin = 50000;
qmax = 180;
qmin = 0;


disp('Solution 1:');
disp([q1 q21 q31 q41]*180/pi);
disp( ([q1 q21 q31]*180/pi)*((smax-smin)/(qmax-qmin)+smin));


disp('Solution 2:');
disp([q1 q22 q32 q42]*180/pi);


%% Draw the arm - this is the forward kinematics.
draw_q1 = [q1   q1]';
draw_q2 = [q21   q22]';
draw_q3 = [q31   q32]';
draw_q4 = [q41   q42]';
draw_q5 = [0 0]';


%% Trigonometric abbreviations
c1 = cos(draw_q1);
c12 = cos(draw_q1+draw_q2);
c2 = cos(draw_q2);
c23 = cos(draw_q2+draw_q3);
c234 = cos(draw_q2+draw_q3+draw_q4);
c3 = cos(draw_q3);
c4 = cos(draw_q4);
c5 = cos(draw_q5);

s1= sin(draw_q1);
s2 = sin(draw_q2);
s23 = sin(draw_q2+draw_q3);
s234 = sin(draw_q2+draw_q3+draw_q4);
s3 = sin(draw_q3);
s32 = s23;
s4 = sin(draw_q4);
s5 = sin(draw_q5);
%% Tip position based on handcalculated T05 matrix
xt = c1.*s234.*d5+c1.*c23.*l3+c1.*c2.*l2;
yt = s1.*s234.*d5+s1.*c23.*l3+s1.*c2.*l2;
zt = -c234.*d5+s2.*l2+s23.*l3;
pt = [ xt yt zt ] ;




%% Tip orientation based on handcalculated T05 matrix
r11 = c1.*c234.*c5+s1.*s5; %corresponds to row 1, column 1 of the rotation matrix
r12 = -c1.*c234.*s5+s1.*c5;
r13 = c1.*s234;
r21 = s1.*c234.*c5-c1.*s5;
r22 = -s1.*c234.*s5-c1.*c5;
r23 = s1.*s234;
r31 = s234.*c5;
r32 = -s234.*s5;
r33 = -c234;

rot1 = [r11(1) r12(1) r13(1);
    r21(1) r22(1) r23(1);
    r31(1) r32(1) r33(1)];% we single out the first set of angles
%inputted at the top so that we can produce a complete numerical transformation function



%% plots up to 4 arm positions in 3d
figure (2)

x1 = zeros(4,1);
y1 = zeros(4,1);
z1 = zeros(4,1);

x2 = zeros(4,1);
y2 = zeros(4,1);
z2 = zeros(4,1);

x3 = c1.*c2.*l2;
y3 = s1.*c2.*l2;
z3 = s2.*l2;

x4 = c1.*c23.*l3+c1.*c2.*l2;
y4 = s1.*c23.*l3+s1.*c2.*l2;
z4 = s23.*l3+s2.*l2;




for i = 1:2
    xx = [ x1(i); x2(i); x3(i); x4(i); pt(i,1) ] ;
    yy = [ y1(i); y2(i); y3(i); y4(i); pt(i,2) ] ;
    zz = [ z1(i); z2(i); z3(i); z4(i); pt(i,3) ] ;
    if i==1
        plot3(xx,yy,zz,'ko-','Linewidth',2,'color', 'red')
    else
        plot3(xx,yy,zz,'ko-','Linewidth',2,'color', 'blue')
        
    end
    axis equal
    hold on
    box on
    grid on
    %   calculates the transformation matrix
    %     rot = [r11(i) r12(i) r13(i);
    %         r21(i) r22(i) r23(i);
    %         r31(i) r32(i) r33(i)];
    %     pos = [ xt(i) yt(i) zt(i) ]' ;
    %
    %
    %     disp([rot pos;0 0 0 1]);
    
    
    xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)') ;
    text(pt(1,1),pt(1,2),'x') ; text(pt(1,1) + 0.1,pt(1,2) + 0.1,pt(1,3) + 0.1,'Test') ;
    pause(0.1)
    
    
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Workspace plots the workspace in 3d, takeing into account the joint
%%restrictions

cw1 = 0;
cw2 = 0;
cw3 = 0;
cw4 = 0;
sw1 = 0;
sw2 = 0;
sw3 = 0;
sw4 = 0;
sw12 = 0;
sw23 = 0;
cw23 = 0;
cw12 = 0;
sw234 = 0;
cw234 = 0;
%
figure (3)

xwork = zeros(70000,1) ; % reserving space for the variables, because
ywork = zeros(70000,1) ; % otherwise they would be created later within a loop.
zwork = zeros(70000,1);
p=0;
for i = -181               % for q1
    qw1 = i*pi/180;
    cw1 = cos(qw1);
    sw1 = sin(qw1);
    
    for j = 1:10:135                       % for q2
        qw2 = j*pi/180;
        cw2 = cos(qw2);
        sw2 = sin(qw2);
        
        for k = -135:10:-10        % for q3
            qw3 = k*pi/180;
            cw3 = cos(qw3);
            sw3 = sin(qw3);
            
            
            qw4 = (pi/2-(qw2+qw3)); %q4
            cw4 = cos(qw4);
            sw4 = sin(qw4);
            
            p=p+1;
            
            cw12   = cos(qw1 + qw2);
            sw12   = sin(qw1 + qw2);
            cw23   = cos(qw2 + qw3);
            sw23   = sin(qw2 + qw3);
            cw234  = cos(qw2 + qw3 + (pi/2-(qw2+qw3)));
            sw234  = sin(qw2 + qw3 + (pi/2-(qw2+qw3)) );
            
            
            xwork(p) = cw1*sw234*d5+cw1*cw23*l3+cw1*cw2*l2;
            ywork(p) = sw1*sw234*d5+sw1*cw23*l3+sw1*cw2*l2;
            zwork(p) = -cw234*d5+sw2*l2+sw23*l3;
            if zwork(p) <= -l1
                zwork(p) = -l1;
            end
            
        end
        
    end
end



plot3(xwork,ywork,zwork,'.')
hold on
box on
grid on

xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)') ;
axis equal




