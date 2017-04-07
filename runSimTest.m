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
u = 1;
u_des =0;
x = 0;
tspan = 0:0.01:400;
dt = 0.01;

%input('Enter the x-coordinate : ');
x_des = 30;
U = zeros(length(tspan),5);
U(1,:)=[x u n x_des-x u_des-u];

%Gains
Kp = 1;
Kd = 10;


%Running Simulations

for i=2:length(tspan)
    
    
    e = x_des-x;  
    e_dot = u_des - u;
    
  if abs(e)<1
    N1 =  (Kp*e + Kd*e_dot)/B1;% + (B/B1)*u*u;
    n = sign(N1)*sqrt(sign(N1)*N1);
  elseif e>=1
          n = 110.969;
      else n = -110.969;
  end
  disp(Kp*e);
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


    
    disp('u=');disp(u);%disp(tspan(i)); 
end

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
 