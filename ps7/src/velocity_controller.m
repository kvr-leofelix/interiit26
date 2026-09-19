function desiredPitch = velocity_controller( ...
    desiredVelocity, actualVelocity, Kp_vel, g)
% VELOCITY_CONTROLLER
%
% Velocity error -> desired pitch angle
%
% Small-angle approximation:
%
% horizontal acceleration ~= g * pitch

velocityError = desiredVelocity - actualVelocity;

desiredAcceleration = ...
    Kp_vel * velocityError;

desiredPitch = ...
    desiredAcceleration / g;

end