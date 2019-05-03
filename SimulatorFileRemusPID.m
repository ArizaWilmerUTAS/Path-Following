%% REMUS Simulator of path following of a helix with LOS and robust PID
%% Script created by:
%% Wilmer Ariza
%% University of Tasmania
%% Please refer to the Github for an explanation of the script

% start variable
clear all;
clc;
T=100;% total time of simulation
dt=0.1; % sample time
time=0:dt:T;%vector of time
OPTIONS = odeset('RelTol', 1e-12, 'AbsTol', 1e-12); %option of the solver
con = functions(@zoh); con = con.function; % ZOH for actuator effect over the system
ctrlfcn = str2func(con); u0 = cell(1,3);        % create control function

%% Path Generation
m=0.6;% variable that stablish the slope of time
Yramp=m*time;%Yramp=slope*time vector

%Desire point generation by the use of paraametric equation of helix
Xellipse=60*(cos(0.02618*Yramp));
Yellipse=60*(sin(0.02618*Yramp));
Zellipse=2+(2*Yramp./200);

%% Initialization values
%=cell(T/dt+1,1);
virtual_point=num2cell([Xellipse;Yellipse;Zellipse],1)';
vehicle_position=cell(T/dt+1,1);
vehicle_position{1}=[60,3,1,0,0,3*pi/4];%initial state
vehicle_velocity=cell(T/dt+1,1);
vehicle_velocity{1}=[0.5,0,0,0,0,0];%initial state
desire_speed=[m;0;0];%desire average speed
point_velocity=num2cell([diff(Xellipse)./diff(time);diff(Yellipse)./diff(time);diff(Zellipse)./diff(time)],1);%[dx/dt;dy/t;dz/dt]
PIDu=pidcreate(1.9,0.1,0,0.1,40,0,dt);%(1.9,0.8,0,0.1,29,-29,dt);
PIDelevator=pidcreate(3.5,0.5,0.3,0.1,15,-15,dt);%(2.65,0.5,0.1,0.1,7,-7,dt);
PIDrudder=pidcreate(0.4,0.3,0,0.1,15,-15,dt);%(0.4,0.3,0,0.1,7,-7,dt)
xdot=zeros(12,1);
x=[cell2mat(vehicle_velocity(1)) cell2mat(vehicle_position(1))];
delay = 0;
tau = dt;
par.dt = dt; par.delay = delay; par.tau = tau;

%start of cycles of simulation
for i=2:T/dt
 % LOS calculation   
[chi_vehicle(i),upsilon_vehicle(i),upsilon_desire(i),chi_desire(i),ud(i)]=LOS(...
    xdot(7:9,1),cell2mat(vehicle_position(i-1)),...
    cell2mat(virtual_point(i-1)),cell2mat(point_velocity(i-1)),...
    desire_speed,time(i-1));
%PID calculation
[RPM,Rudderangle,elevatorangle,PIDu,PIDelevator,PIDrudder]=LOSPID(chi_vehicle(i),upsilon_vehicle(i),upsilon_desire(i),chi_desire(i),ud(i),cell2mat(vehicle_position(i-1)),cell2mat(vehicle_velocity(i-1)),PIDu,PIDelevator,PIDrudder);

ui(i,:)=[RPM,elevatorangle,Rudderangle];
%calcualtion of errors
error_chi(i)=chi_desire(i)-chi_vehicle(i);
error_upsilon(i)=upsilon_desire(i)-upsilon_vehicle(i);
ax=cell2mat(vehicle_velocity(i-1));
error_u(i)=ud(i)-ax(1);
%x=[cell2mat(vehicle_velocity(i-1)) cell2mat(vehicle_position(i-1))];

%vehicle simulation
U=ui(i,:)';
for j = 1:3
    u0{j} = @(t)ctrlfcn(U(j,:),t,par);
end
dynamics=@REMUS;
% xdot= MULLAYA(x,ui(i,:));
% x = euler2(xdot',x,dt);
[T,y] = ode45(dynamics, [0 dt/2 dt], x, OPTIONS,u0{:});
x1 = y(3,:)'; 
%calculation of the derivative of the state
xdot=(x1-x')/dt;
x=x1';
% next state 
vehicle_velocity{i}=x(1,1:6);
vehicle_position{i}=x(1,7:12);
end
vehicle_position=cell2mat(vehicle_position);
particle_position=cell2mat(virtual_point')';
vehicle_velocity=cell2mat(vehicle_velocity);
figH = figure(1);
set(figH,'Name','Position','NumberTitle','off')
subplot(3,1,1);
plot(time,particle_position(:,1));hold on
plot(time(:,1:end-1),vehicle_position(:,1));hold off
title('X');
subplot(3,1,2);
plot(time,particle_position(:,2));hold on
plot(time(:,1:end-1),vehicle_position(:,2));hold off
title('Y');
subplot(3,1,3);
plot(time,particle_position(:,3));hold on
plot(time(:,1:end-1),vehicle_position(:,3));hold off
title('Z');

figH = figure(2);
set(figH,'Name','Inputs','NumberTitle','off')
subplot(3,1,1);
plot(time(:,1:end-1),ui(:,1));hold off
title('RPM');
subplot(3,1,2);
plot(time(:,1:end-1),ui(:,2));hold off
title('Elevator');
subplot(3,1,3);
plot(time(:,1:end-1),ui(:,3));hold off
title('Rudder');


figH = figure(3);
set(figH,'Name','LOS','NumberTitle','off')
subplot(3,1,1);

plot(time(:,1:end-1),chi_vehicle);hold on
plot(time(:,1:end-1),chi_desire);hold off
title('\bf\chi');
subplot(3,1,2);

plot(time(:,1:end-1),upsilon_vehicle);hold on
plot(time(:,1:end-1),upsilon_desire);hold off
title('\bf\upsilon');
subplot(3,1,3);

plot(time(:,1:end-1),vehicle_velocity(:,1));hold on
plot(time(:,1:end-1),ud);hold off
title('\bf\mu');

figH = figure(4);
set(figH,'Name','Position3D','NumberTitle','off')

plot3(particle_position(:,1),particle_position(:,2),particle_position(:,3));hold on
plot3(vehicle_position(:,1),vehicle_position(:,2),vehicle_position(:,3));
% daspect([1,1,1]);
hold off



figH = figure(5);
set(figH,'Name','LOS','NumberTitle','off')
subplot(3,1,1);

plot(time(:,1:end-1),error_chi);
title('\bf\chi');
subplot(3,1,2);

plot(time(:,1:end-1),error_upsilon);
title('\bf\upsilon');
subplot(3,1,3);

plot(time(:,1:end-1),error_u);
title('\bf\mu');


function u = zoh(f, t, par) % **************************** zero-order hold
d = par.delay;
if d==0
    u = f;
else
    e = d/100; t0=t-(d-e/2);
    if t<d-e/2,     u=f(1);
    elseif t<d+e/2, u=(1-t0/e)*f(1) + t0/e*f(2);    % prevents ODE stiffness
    else            u=f(2);
    end
end
end




