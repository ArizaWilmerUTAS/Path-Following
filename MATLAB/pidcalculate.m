function pid=pidcalculate(pid,error)
% calculate PID
if pid.counter==0
    pid.error = error;
    pid.integral = double(pid.integral + (pid.error*pid.iteration_time));
    derivative = (pid.error-pid.error_prior)/pid.iteration_time;
    pid.output = pid.Kp*pid.error + pid.Ki*pid.integral + pid.Kd*derivative;
    pid.output=round(pid.output,3);
    if pid.output<pid.LOWER_limit
        pid.output=pid.LOWER_limit;
    end
    if pid.output>pid.UPPER_limit
        pid.output=pid.UPPER_limit;
    end
    pid.error_prior = pid.error;
    pid.counter=pid.counter+1;
elseif pid.counter==pid.counter_cyles_time
    pid.counter=0;
else
    pid.counter=pid.counter+1;
end
    %pause(pid.iteration_time);
    
end