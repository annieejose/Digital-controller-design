clear all 
clc 
close all
syms z 
global K1  K2 ref_1 ref_2 K3 ref_3 ref_4 

%Conversion from transfer function into continuous time state space model 
num = [61.63 0 21052.808]; 
den = [1 40.32 965.3031 13774.537 0]; 
tfc = tf(num,den) 
[A,B,C,D] = tf2ss(num,den) 
sysc = ss(A,B,C,D); 
cltf_sysc = feedback(sysc,1);

% figure(1) 
% step(sysc); 
% axis([0 100 0 100]) 
% figure(2) 
% pzmap(sysc); 
% figure(3) 
% pzmap(cltf_sysc); 

%Discretized state space model 
T = 0.026; 
sysd = c2d(sysc,T,'zoh') 
[a,b,c,d] = ssdata(sysd)  
tfd =tf(sysd)  
% figure(4) 
% pzmap(sysd); 
% figure(5) 
% step(sysd); 

%Regulator problem  
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

W=ctrb(a,b);
if rank(a)==rank(W) 
disp('System is controllable')
K1 = place(a,b,p) 
A_a = a-b*K1; 
sys_new = ss(A_a,b,c,d,T); 
% figure(6) 
% step(sys_new); 
% figure(7) 
% pzmap(sys_new) 
t0_a = 1; 
tf_a = 100; 
x0_a=[2 ;0; 2; 0]; 
XARRAY_a = []; 
UARRAY_a = []; 
KARRAY_a = []; 
X=x0_a; 
for k=t0_a:1:tf_a 
[X,U,Y] = solvediff(X,a,b,c,d); 
XARRAY_a(k,:) = reshape(X,1,4); 
UARRAY_a(k,:) = U; 
KARRAY_a(k,:) = k; 
end 
figure(8) 
stem(KARRAY_a,XARRAY_a,'LineWidth',1.5); grid on;
xlabel('time (s)');
ylabel('System states');
legend('X1','X2','X3','X4'); 
title("Time variation of system states for regulator controller");
axis([0 100 -150 150]); 
else
disp('System is Not controllable');
end 
%Servo controller without integral states 
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

W=ctrb(a,b);
if rank(a)==rank(W) 
disp('System is controllable')
K2 = place(a,b,p) 
A_a = a-b*K2; 
sys_new = ss(A_a,b,c,d,T); 
% figure(9) 
% step(sys_new); 
% figure(10) 
% pzmap(sys_new) 
t0_b = 1; 
tf_b = 200; 
x0_b=[0.2 ;0; 0.2; 0]; 
ref_1 = 20 
ref_2 = 40 
N_inv = -c*inv(a - b*K2 - eye(4))*b; 
N = inv(N_inv); 
XARRAY_b = []; 
UARRAY_b = []; 
KARRAY_b = []; 
YARRAY_b = []; 
ERROR_b = [];
X=x0_b; 
for k=t0_b:1:tf_b 
[X,U,Y,E] = solvediffs(X,N,a,b,c,d,k); 
XARRAY_b(k,:) = reshape(X,1,4); 
UARRAY_b(k,:) = U; 
KARRAY_b(k,:) = k; 
YARRAY_b(k,:) = Y; 
ERROR_b(k,:) = E;
end 
figure(11) 
stem(KARRAY_b,XARRAY_b,'LineWidth',1.5),legend('X1','X2','X3','X4'); grid on;
xlabel('time (s)');
ylabel('System states');
title("Variation of states for servo control without integral states");
axis([0 150 -50 70]) 
figure(12) 
stem(KARRAY_b,YARRAY_b,'LineWidth',1.5);grid on; 
title("System output with time for servo control without integral states");
xlabel('time (s)');
ylabel('System output');
axis([0 200 -50 70]) 
else
disp('System is Not controllable');
end 

%Servo controller with integral states 
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
z5 = 0.0947 
p = [z1 z2 z3 z4 z5] 
% Augmented system with integral states 
F = [a zeros(4,1) 
c*a ones(1,1)] 
g = [b 
c*b] 
c = [c 0] 
W=ctrb(F,g);
if rank(F)==rank(W) 
disp('System is controllable')
K3 = place(F,g,p) 
A_a = F - g*K3; 
sys_new = ss(A_a,g,c,d,T); 
% figure(13) 
% step(sys_new); 
% figure(14) 
% pzmap(sys_new) 
t0_c = 1; 
tf_c = 200; 
x0_c=[0.2 ;0; 0.2; 0 ;0]; 
ref_3 =20; 
ref_4 = 40; 
XARRAY_c = []; 
UARRAY_c = []; 
KARRAY_c = [];
ERROR_c = [];
X=x0_c; 
for k=t0_c:1:tf_c 
[X,U,Y,E] = solvediffsi(X,F,g,c,d,k); 
XARRAY_c(k,:) = reshape(X,1,5); 
UARRAY_c(k,:) = U; 
KARRAY_c(k,:) = k; 
YARRAY_c(k,:) = Y; 
ERROR_c(k,:) = E;
end 

figure(15) 
stem(KARRAY_c,XARRAY_c(:,1:4),'LineWidth',1.5),legend('X1','X2','X3','X4'); grid on;
xlabel('time (s)');
ylabel('System states');
title("Variation of system states for servo control with integral states")
axis([0 150 -50 70]); 
figure(16) 
stem(KARRAY_c,YARRAY_c,'LineWidth',1.5); grid on;
xlabel('time (s)');
ylabel('System output');
title("System output  with time for for servo control with integral states")
axis([0 200 -50 70]) 
figure(17)
plot(KARRAY_b,YARRAY_b,'LineWidth',1.5);grid on;hold on;
xlabel('time (s)');
ylabel('System output');
plot(KARRAY_c,YARRAY_c,'LineWidth',1.5);grid on;
axis([0 200 -50 70]);
title('System output for servo with ref 20 and 40(after 130 sec)')
legend('Servo without integral','Servo with integral');
else
disp('System is Not controllable');
end

function [xdiff,u,y ]= solvediff(x,a,b,c,d) 
global K1   
xdiff=zeros(4,1); 
xdiff = (a-b*K1)*x; 
u = -K1*x; 
y = c*x; 
end 

function [xdiff,u,y,e ]= solvediffs(x,N,a2,b2,c2,d2,k) 
global K2 ref_1 ref_2 
xdiff=zeros(4,1); 
if k<130
    ref = ref_1;
else
    ref = ref_2;
end
u = -K2*x+N*ref;    
xdiff = a2*x+b2*u; 
y = c2*x;
e = y-ref;
end 

function [xdiff,u ,y,e]= solvediffsi(x,F,g,c,d2,k) 
global K3 ref_3 ref_4 
xdiff=zeros(5,1); 
u = -K3*(x); 
l = [0;0;0;0;-1]; 
 if k<130
     ref = ref_3;
 else
     ref = ref_4;
 end
xdiff = F*x+g*u+l*ref; 
y = c*x; 
e = y-ref;
end 