
  function runSimTest()


  clc;
  close all;
  addpath('actuator dynamics');
  addpath('utils');  
  addpath('PDcontrol');
  addpath('Solvers');
  addpath('excel data');
  addpath('excel data/u_1m_s');
  addpath('excel data/u_1dot5m_s');
  addpath('excel data/u_2m_s');
  geoprop;
  surgederivatives;
%Initializations
B = 0.5*rho*L*L*0.00385/(m-(0.5*rho*L*L*L*surge_deriv(5)));
B1 = B*.012*.012;
 
n = 0;
u = .5;
u_des =0;
x = 0;
tspan = 0:0.01:100;
dt = 0.01;

x_des = 30;%input('Enter the x-coordinate : ');
U = zeros(length(tspan),5);
U(1,:)=[x u n x_des-x u_des-u];

%Gains
Kp = .10;
Kd = 0;

%Dynamics
% eta = 0.012*n/u;
% u_dot = A*u*u*(eta*abs(eta)-1);

%disp(u_dot);

%Controller
% u_dot_des = u_dot + Kp*e + Kd*e_dot;
% 
% if u_dot_des > 0
%        n = sqrt(abs(((u_dot_des/A)+u*u)/(0.012*0.012)));
%        if n<83.33*u, n=83.33*u; end
%    elseif u_dot_des<0
%        n = -1*sqrt(abs(-1*((u_dot_des/A)+u*u)/(0.012*0.012)));
%        if n<-248.6*u, n=-248.6*u;
%        elseif n>0, n=0; end
% end

%cap on n
%if n>200, n =200; elseif n<-200,n =-200;end;

%Running Simulations

for i=2:length(tspan)
    
    
    e = x_des-x;  
    e_dot = u_des - u;
    N1 =  (Kp*e + Kd*e_dot)/B1;% + (B/B1)*u*u;
    n = sign(N1)*sqrt(sign(N1)*N1);
    
    if n>200
       n = 200; 
    elseif n<-200
            n = -200;
    end
    
    eta = 0.012*n/u;
    u_dot = B*u*u*(eta*abs(eta)-1);
    
    x = x + u*dt;
    u = u + u_dot*dt;
    %if u < 0.05, u = 0.000001; end
    U(i,:) = [x u n e e_dot];
%     
%     if u_dot_des > 0
%         n = sqrt(((u_dot_des/A)+u*u)/(0.012*0.012));
%         if n<83.33*u, n=83.33*u; end
%     elseif u_dot_des<0
%         n = -1*sqrt(-1*((u_dot_des/A)+u*u)/(0.012*0.012));
%         if n<-248.6, n=83.33*u; end
%     end
%    if u_dot_des > 0
%        n = sqrt(abs(((u_dot_des/A)+u*u)/(0.012*0.012)));
%        if n<83.33*u, n=83.33*u; end
%    elseif u_dot_des<0
%        n = -1*sqrt(abs(-1*((u_dot_des/A)+u*u)/(0.012*0.012)));
%        if n<-248.6*u, n=-248.6*u; end
%        if n>0, n=0; end
%    end
       %cap on n
%if n>200, n =200; elseif n<-200,n =-200;end;


    
    disp(u);%disp(tspan(i)); 
end
%If u_dot_des > 0, we assume Phase I: Acceleration ==> eta>0

%Plots
figure;
  subplot(3,1,1),plot(tspan,U(:,1));xlabel('t');ylabel('x');
  subplot(3,1,2),plot(tspan,U(:,2));xlabel('t');ylabel('u');
  subplot(3,1,3),plot(tspan,U(:,3));xlabel('t');ylabel('n');
   display('simulation done');
   
   figure;
   subplot(2,1,1),plot(tspan,U(:,4));xlabel('t');ylabel('e');
  subplot(2,1,2),plot(tspan,U(:,5));xlabel('t');ylabel('e_dot');
  
   %plot(tspan, U(:,3)./U(:,2));
   
  end
 
  