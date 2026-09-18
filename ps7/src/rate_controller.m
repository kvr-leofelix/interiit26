function torque = rate_controller(rate_desired, rate_actual, Kp, Ki, Kd, integral_error, prev_error, dt)
% RATE_CONTROLLER
% PID controller for roll, pitch and yaw angular rates.
%
% Inputs:
% rate_desired  = desired [p q r]
% rate_actual   = measured [p q r]
% Kp, Ki, Kd    = PID gain vectors
% integral_error = accumulated error
% prev_error     = previous error
% dt             = controller timestep
%
% Output:
% torque = requested [roll pitch yaw] torque [N*m]

error = rate_desired - rate_actual;

derivative = (error - prev_error) / dt;

torque = Kp .* error ...
       + Ki .* integral_error ...
       + Kd .* derivative;

end