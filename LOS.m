function [chi_vehicle,upsilon_vehicle,upsilon_desire,chi_desire,ud]=LOS(Vehicle_velocity,vehicle_position,virtual_point,point_velocity,desire_speed,time_to_point)
% Vehicle_velocity: vehicle vector of velocity [x_dot,y_dot,z_dot]
% vehicle_position: position vector[X,Y,Z]
% virtual_point; virtual point position vector[X,Y,Z]
% time_to_point= step time
%desire speed vector [u,v,w]
%desire_velocity Ud
chi_vehicle=0;
upsilon_vehicle=0;
upsilon_desire=0;
chi_desire=0;
ud=0.5;
% orientation vehicle asimuth and elevation
chi_vehicle=atan2(Vehicle_velocity(2),Vehicle_velocity(1));

if chi_vehicle< pi
    chi_vehicle = chi_vehicle;
else
    chi_vehicle = chi_vehicle-pi;
end
if Vehicle_velocity==[0,0,0]'
    upsilon_vehicle=atan(0);
else
upsilon_vehicle=atan(-Vehicle_velocity(3)/sqrt(Vehicle_velocity(2)^2+Vehicle_velocity(1)^2));
end
% orientation point
virtual_point_velocity=point_velocity;
% if time_to_point==0
%     virtual_point_velocity=[0,0,0]' ;
%     psi_point=0;
%     theta_point=0;
% else
    psi_point=atan2(virtual_point_velocity(2),virtual_point_velocity(1));
    theta_point=atan(-virtual_point_velocity(3)/sqrt(virtual_point_velocity(2)^2+virtual_point_velocity(1)^2));
% end

x=vehicle_position(1);
y=vehicle_position(2);
z=vehicle_position(3);
x_p=virtual_point(1);
y_p=virtual_point(2);
z_p=virtual_point(3);
% error vector
x_e=cos(psi_point)*cos(theta_point)*(x-x_p)+sin(psi_point)*cos(theta_point)*(y-y_p)-sin(theta_point)*(z-z_p);
y_e=-sin(psi_point)*(x-x_p)+cos(psi_point)*(y-y_p);
z_e=cos(psi_point)*sin(theta_point)*(x-x_p)+sin(psi_point)*sin(theta_point)*(y-y_p)+cos(theta_point)*(z-z_p);

% LOS Guidance angles
ky=1;
kz=1;
deltay=150;
deltaz=150;
psi_r=tanh(-ky*y_e/deltay);
theta_r=tanh(kz*z_e/deltaz);
u=desire_speed(1);
v=desire_speed(2);
w=desire_speed(3);

% Guidance Law
upsilon_desire=asin(sin(theta_point)*cos(theta_r)*cos(psi_r)+cos(theta_point)*sin(theta_r));
chi_y=cos(psi_point)*sin(psi_r)*cos(theta_r)-sin(theta_point)*sin(theta_r)*sin(psi_point)+sin(psi_point)*cos(psi_r)*cos(theta_point)*cos(theta_r);
chi_x=-sin(psi_point)*sin(psi_r)*cos(theta_r)-sin(theta_point)*sin(theta_r)*cos(psi_point)+cos(psi_point)*cos(psi_r)*cos(theta_point)*cos(theta_r);
chi_desire=atan2(chi_y,chi_x);
% if time_to_point==0
%     chi_desire=0;
%     upsilon_desire=0;
% end





% % k=1;

if time_to_point==0
  desire_velocity=desire_speed(1)*1.5;
else
    desire_velocity=(norm(virtual_point_velocity)-0.001*x_e)/(cos(psi_r)*cos(theta_r));
end

if desire_velocity>=2.5
    desire_velocity=2.5;
end
if desire_velocity<=0
    desire_velocity=0;
end
ud=desire_velocity*u/sqrt(u^2+v^2+w^2);
%ud=1.5;
a=isnan(ud);

end