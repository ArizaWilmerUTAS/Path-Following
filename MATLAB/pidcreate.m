%% Create a PID with the respective variables
% Inputs:
%   Kp= Proportional value
%   Kd= Derivative value
%   Ki= integraal value
%   Ts= sample time
%   UPPER_limit= max value
%   LOWER_limit=min value
%   simulation_time
% Output
%  PIDstrucure
function name=pidcreate(Kp,Kd,Ki,Ts,UPPER_limit,LOWER_limit,simulation_time)

name.PID=1;
name.Kp=Kp;                              % proportional controller gain
name.Kd=Kd;                               % derivative controller gain
name.Ki=Ki;                               % integral controller gain
name.desired_value = 0;                              % desired angular velocity (krpm)
name.integral = double(0);                               % initialize error integral to zero
name.error = double(0);                             % initialize past values of error
name.error_prior = double(0);                             % to zero
name.UPPER_limit=UPPER_limit;
name.LOWER_limit=LOWER_limit;
name.iteration_time=Ts;
name.counter_cyles_time=Ts/simulation_time;
name.counter=0;
end