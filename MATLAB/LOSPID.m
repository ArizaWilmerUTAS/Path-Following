function [RPM,Rudderangle,elevatorangle,PIDu,PIDelevator,PIDrudder]=LOSPID(chi_vehicle,...
    upsilon_vehicle,upsilon_desire,chi_desire,ud,position,velocity,...
    PIDu,PIDelevator,PIDrudder)
%% Calculate PID value
% Input
%  chi_vehicle=angle vehicle from LOS
%  upsilon_vehicle=angle vehicle from LOS
%  upsilon_desire=angle point from LOS
%  chi_desire=angle point from LOS
%  ud= desire vehicle velocity
%  position=vehicle position
%  velocity=vehicle velocity
%  PIDu,PIDelevator,PIDrudder= PID for each surface
% Output
%  RPM=propeller RPM,
%  Rudderangle=rudder force
%  elevatorangle=elevator force
phi=position(4);
theta=position(5);
psi=position(6);
u=velocity(1);
v=velocity(2);
w=velocity(3);
p=velocity(4);
q=velocity(5);
r=velocity(6);

error_chi=chi_desire-chi_vehicle;
error_upsilon=upsilon_desire-upsilon_vehicle;
error_u=ud-u;
error_chi=atan2(sin(error_chi),cos(error_chi));
error_upsilon=atan2(sin(error_upsilon),cos(error_upsilon));
% error_u = error_u + randn(1)*chol(0.02^2);
% error_chi = error_chi + randn(1)*chol(0.005^2);
% error_upsilon = error_upsilon+ randn(1)*chol(0.005^2);
PIDu=pidcalculate(PIDu,error_u);
RPM=-54.04*v*r+54.04*w*q+24.91*PIDu.output;
if RPM<PIDu.LOWER_limit
    RPM=PIDu.LOWER_limit;
elseif RPM>PIDu.UPPER_limit
    RPM=PIDu.UPPER_limit;
end
PIDelevator=pidcalculate(PIDelevator,error_upsilon);
elevatorangle=(-54.04*u*w+9.81*1.96e-2*sin(theta)+6.95*q+24.91*PIDelevator.output);
if elevatorangle<PIDelevator.LOWER_limit
    elevatorangle=PIDelevator.LOWER_limit;
elseif elevatorangle>PIDelevator.UPPER_limit
    elevatorangle=PIDelevator.UPPER_limit;
end
PIDrudder=pidcalculate(PIDrudder,error_chi);

Rudderangle=29.128*u*v+8.1152*PIDrudder.output+6.995*r;
if Rudderangle<PIDrudder.LOWER_limit
    Rudderangle=PIDrudder.LOWER_limit;
elseif Rudderangle>PIDrudder.UPPER_limit
    Rudderangle=PIDrudder.UPPER_limit;
end

end