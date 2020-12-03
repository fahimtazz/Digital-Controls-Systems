A= [0 0 1 0; 0 0 0 1;0 767.05 -1801.41 0; 0 1040.06 1801.41 0];
B= [0; 0; 3355.21; -3355.21]
C= [1 1 0 0];
D=0;
Ts= 1e-03;
K=368;
sys= ss(A,B,C,D)%State Space model of flexibke joint
sys2= c2d(sys,Ts)%discrete State Spaec Model
Co=ctrb(A,B)%Controllability
rank(Co)
Ob=obsv(A,C)%Observability
rank(Ob)
zeta=0.6;
wn=20*Ts;
figure(1)
rlocus(sys2)
axis([-1 1 -1 1])
zgrid(zeta,wn)
% [k,poles]= rlocfind(sys2)
%%k = 355.0764


%poles =

 %  1.1141 + 0.0000i
  % 0.9438 + 0.0856i
   %0.9438 - 0.0856i
   %0.1650 + 0.0000i
   %I used the commented steps above to find the poles 
  p=[1.1141+0i 0.9438+0.0856i 0.9438-0.0856i 0.1650+0i];
   L= acker(A,B,p) %State feedback gain
   %L =-0.0000   -0.3110   -0.5369    0.0009
   P=[-0.2 -.21 -.22 -.23]; %slowest observer pole
   Obs=acker(sys2.A',sys2.C',P/10) %Full order observer 10 times faster than slowest pole
% rsys= reg(sys2,L,Obs')%step6: trying to close the feedback loop
%    step(rsys)
%    X= dlyap(sys2.A+B*L,eye(4))%Lyapunov Stability Analysis
   sys_cl= ss(sys2.A-sys2.B*L,sys2.B,sys2.C,sys2.D,Ts)
   figure(2)
   step(sys_cl)
   stepinfo(sys_cl)
   sys_obs=ss([sys2.A-sys2.B*L sys2.B.*L; zeros(size(sys2.A)) sys2.A-Obs.*sys2.C],[sys2.B;zeros(size(sys2.B))],[sys2.C zeros(size(sys2.C))],sys2.D,Ts)
   figure(3)
   step(sys_obs)
   stepinfo(sys_obs)
