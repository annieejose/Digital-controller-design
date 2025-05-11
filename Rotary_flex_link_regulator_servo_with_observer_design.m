clear all 
clc 
close all
syms z 
global K4 K5 ref_5 ref_6 

%Conversion from transfer function into continuous time state space model 
num = [61.63 0 21052.808]; 
den = [1 40.32 965.3031 13774.537 0]; 
tfc = tf(num,den) 
[A,B,C,D] = tf2ss(num,den) 
sysc = ss(A,B,C,D); 
% figure(1) 
% step(sysc); 
% axis([0 100 0 100]) 
% figure(2) 
% pzmap(sysc);

%Discretized state space model 
T = 0.026; 
sysd = c2d(sysc,T,'zoh') 
[a,b,c,d] = ssdata(sysd)  
tfd =tf(sysd)  
% figure(3) 
% pzmap(sysd); 
% figure(4) 
% step(sysd); 

%Controller design  
m_p = 0.1; 
t_s = 2 
Seta = sqrt((log(m_p))^2/(pi^2+(log(m_p))^2)) 
Wn = 4/(Seta*t_s) 
s1 = -Seta*Wn+i*sqrt(1-Seta^2)*Wn; 
s2 = -Seta*Wn-i*sqrt(1-Seta^2)*Wn; 
z1 = exp(s1*T) 
z2 = exp(s2*T) 
z3 = z1/10 
z4 = z2/10 
p = [z1 z2 z3 z4] 
K4 = place(a,b,p) 
A_a = a-b*K4; 
sys_new = ss(A_a,b,c,d,T); 
% figure(5) 
% step(sys_new); 
% figure(6) 
% pzmap(sys_new) 

%Observer design 
t_so = 1; 
m_po = 0.1; 
Seta_o = sqrt((log(m_po))^2/(pi^2+(log(m_po))^2)) 
Wn_o = 4/(Seta_o*t_so) 
s1 = -Seta_o*Wn_o+i*sqrt(1-Seta_o^2)*Wn_o; 
s2 = -Seta_o*Wn_o-i*sqrt(1-Seta_o^2)*Wn_o; 
z1 = exp(s1*T) 
z2 = exp(s2*T) 
z3 = z1/10 
z4 = z2/10 
p = [z1 z2 z3 z4]

V=obsv(a,c); 
W=ctrb(a,b);
if rank(a)==rank(V) && rank(a)==rank(W) 
disp('System is Observable and controllable')
m = place(transpose(a),transpose(c),p); 
m= transpose(m); 
A_a = a-m*c; 
sys_new = ss(A_a,b,c,d,T); 
% figure(7) 
% step(sys_new); 
% figure(8) 
% pzmap(sys_new) 
t0_a = 1; 
tf_a = 200; 
x0_a=[0.2 ;0; 0.2; 0]; 
XARRAY_a = []; 
UARRAY_a = []; 
KARRAY_a = []; 
X=x0_a; 
X_CAP = x0_a; 
for k=t0_a:1:tf_a 
[X,X_CAP,U,Y,E] = solvediffo(X,X_CAP,a,b,c,d,m); 
XARRAY_a(k,:) = reshape(X,1,4); 
X_CAPARRAY_a(k,:) = reshape(X_CAP,1,4); 
UARRAY_a(k,:) = U; 
KARRAY_a(k,:) = k; 
EARRAY_a(k,:) = E; 
YARRAY_a(k,:) = Y;
end 

figure(9),subplot(2,1,1),plot(KARRAY_a,XARRAY_a,'LineWidth',1.5),title('Actual states'),legend('X1','X2','X3','X4');  grid on;
axis([0 100 -5 5]); 
xlabel('time (s)');
ylabel('System states');
figure(9),subplot(2,1,2),plot(KARRAY_a,X_CAPARRAY_a,'LineWidth',1.5),title('Estimated states'),legend('X1','X2','X3','X4');  grid on;
axis([0 100 -5 5]); 
xlabel('time (s)');
ylabel('System states');
figure(10),plot(KARRAY_a,YARRAY_a,'LineWidth',1.5),title('Variation of output with time for regulator problem'); grid on; 
axis([0 200 -50 50]); 
xlabel('time (s)');
ylabel('System output');

else
   disp('System is Not Observable/controllable') 
end  

% Servo controller without integral states  
%Controller design 
m_p = 0.1; 
t_s = 2 
Seta = sqrt((log(m_p))^2/(pi^2+(log(m_p))^2)) 
Wn = 4/(Seta*t_s) 
s1 = -Seta*Wn+i*sqrt(1-Seta^2)*Wn; 
s2 = -Seta*Wn-i*sqrt(1-Seta^2)*Wn; 
z1 = exp(s1*T) 
z2 = exp(s2*T) 
z3 = z1/10 
z4 = z2/10 
p = [z1 z2 z3 z4] 
K5 = place(a,b,p) 
A_a = a-b*K5; 
sys_new = ss(A_a,b,c,d,T); 
% figure(10) 
% step(sys_new); 
% figure(10) 
% pzmap(sys_new) 

%Observer design 
t_so = 0.5; 
m_po = 0.1; 
Seta_o = sqrt((log(m_po))^2/(pi^2+(log(m_po))^2)) 
Wn_o = 4/(Seta_o*t_so) 
s1 = -Seta_o*Wn_o+i*sqrt(1-Seta_o^2)*Wn_o; 
s2 = -Seta_o*Wn_o-i*sqrt(1-Seta_o^2)*Wn_o; 
z1 = exp(s1*T) 
z2 = exp(s2*T) 
z3 = z1/10 
z4 = z2/10 
p = [z1 z2 z3 z4] 

V=obsv(a,c); 
W=ctrb(a,b);
if rank(a)==rank(V) && rank(a)==rank(W) 
disp('System is Observable and controllable') 
m = place(transpose(a),transpose(c),p); 
m= transpose(m); 
A_a = a-m*c; 
sys_new = ss(A_a,b,c,d,T); 
% figure(11) 
% step(sys_new); 
% figure(12) 
% pzmap(sys_new) 
ref_5 = 5; 
ref_6 = 10; 
N_inv = -c*inv(a - b*K5 - eye(4))*b; 
N = inv(N_inv); 
t0_b = 1; 
tf_b = 300; 
x0_b=[0.2 ;0; 0.2; 0]; 
XARRAY_b = []; 
UARRAY_b= []; 
KARRAY_b = []; 
X=x0_b; 
X_CAP = x0_b; 

for k=t0_b:1:tf_b 
[X,X_CAP,U,Y,E] = solvediffso(X,X_CAP,N,a,b,c,d,m,k); 
XARRAY_b(k,:) = reshape(X,1,4); 
X_CAPARRAY_b(k,:) = reshape(X_CAP,1,4); 
UARRAY_b(k,:) = U; 
KARRAY_b(k,:) = k; 
EARRAY_b(k,:) = E; 
YARRAY_b(k,:) = Y; 
end 



figure(13),subplot(2,1,1),plot(KARRAY_b,XARRAY_b,'LineWidth',1.5),title('Actual states'),legend('X1','X2','X3','X4'); grid on; 
axis([0 300 -5 5]); 
xlabel('time (s)');
ylabel('System states');
figure(13),subplot(2,1,2),plot(KARRAY_b,X_CAPARRAY_b,'LineWidth',1.5),title('Estimated states'),legend('X1','X2','X3','X4'); grid on; 
axis([0 300 -5 5]); 
xlabel('time (s)');
ylabel('System states');
figure(14),plot(KARRAY_b,YARRAY_b,'LineWidth',1.5),title('Variation of output with time for servo with ref of 5 and 10(after 200sec)'); grid on;
axis([0 300 -50 50]); 
xlabel('time (s)');
ylabel('System output');

else
disp('System is Not Observable/controllable') 
end     

%Servo controller with integral states 

m_p = 0.1; 
t_s = 2 ;
Seta = sqrt((log(m_p))^2/(pi^2+(log(m_p))^2)) 
Wn = 4/(Seta*t_s) 
s1 = -Seta*Wn+i*sqrt(1-Seta^2)*Wn; 
s2 = -Seta*Wn-i*sqrt(1-Seta^2)*Wn; 
z1 = exp(s1*T) 
z2 = exp(s2*T) 
z3 = z1/10 
z4 = z2/10 
z5 = 0.0947 
p = [z1 z2 z3 z4 z5] 

% Augmented system with integral states 
F = [a zeros(4,1) 
c*a ones(1,1)] 
g = [b 
c*b] 
c = [c 0]; 
K31 = place(F,g,p) 
A_a = F - g*K31; 
sys_new = ss(A_a,g,c,d,T);

% figure(15) 
% step(sys_new); 
% figure(16) 
% pzmap(sys_new) 

%Observer design 
t_so = 0.5; 
m_po = 0.1; 
Seta_o = sqrt((log(m_po))^2/(pi^2+(log(m_po))^2)) 
Wn_o = 4/(Seta_o*t_so) 
s1 = -Seta_o*Wn_o+i*sqrt(1-Seta_o^2)*Wn_o; 
s2 = -Seta_o*Wn_o-i*sqrt(1-Seta_o^2)*Wn_o; 
z1 = exp(s1*T) 
z2 = exp(s2*T) 
z3 = z1/10 
z4 = z2/10 
z5 = 0.001 
p = [z1 z2 z3 z4 z5] 

V=obsv(F,c); 
W=ctrb(F,g);
if rank(F)==rank(V) && rank(F)==rank(W) 
disp('System is Observable and controllable') 
m = place(transpose(F),transpose(c),p); 
m= transpose(m); 
t0 = 1; 
tf = 300; 
x0=[0 ;0; 0; 0 ;0]; 
ref2o =5; 
XARRAY = []; 
UARRAY = []; 
KARRAY = []; 
X=x0; 
X_CAP = x0; 

for k=t0:1:tf 
[X,X_CAP,U,Y,E] = solvediffsio(X,X_CAP,F,g,c,d,m); 
XARRAY(k,:) = reshape(X,1,5); 
X_CAPARRAY(k,:) = reshape(X_CAP,1,5); 
UARRAY(k,:) = U; 
KARRAY(k,:) = k; 
EARRAY(k,:) = E; 
YARRAY(k,:) = Y; 
end 


figure(17),subplot(2,1,1),plot(KARRAY,XARRAY,'LineWidth',1.5); 
figure(18),subplot(2,1,2),plot(KARRAY,X_CAPARRAY,'LineWidth',1.5); 
figure(19),plot(KARRAY,YARRAY,'LineWidth',1.5);

else 
disp('System is Not Observable/controllable with augmented states') 
end 

function [xdiff,xcapdiff,u,y,E ]= solvediffo(x,xcap,a,b,c,d,m) 
global K4   
xdiff=zeros(4,1); 
xdiff = a*x - b*K4*xcap; 
u = -K4*xcap; 
y = c*x; 
xcapdiff = (a-m*c-b*K4)*xcap+m*y; 
E = xdiff - xcapdiff; 
end 

function [xdiff,xcapdiff,u,y,E ]= solvediffso(x,xcap,N,a2,b2,c2,d2,m,k) 
global K5 ref_5 ref_6 
xdiff=zeros(4,1); 
if k<200 
u = -K5*xcap+N*ref_5; 
else  
u = -K5*xcap+N*ref_6; 
end 
xdiff = a2*x+b2*u; 
y = c2*x; 
xcapdiff = (a2-m*c2)*xcap+b2*u+m*y; 
E = xdiff - xcapdiff; 
end 

function [xdiff,xcapdiff,u,y,E ]= solvediffsio(x,xcap,F,g,c,d2,m) 
global K31 ref2o 
xdiff=zeros(5,1); 
u = -K31*(xcap); 
l = [0;0;0;0;-1]; 
xdiff = F*x + g*u + l*ref2o; 
y = c*xdiff; 
xcapdiff = (F-m*c)*xcap+g*u+m*y; 
E = xdiff - xcapdiff; 
end