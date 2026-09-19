function desiredVelocity = position_controller(desiredPosition, actualPosition, Kp_pos)
% POSITION_CONTROLLER
%
% Position error -> desired velocity
%
% Example:
% We want x = 5 m
% We are at x = 2 m
% Controller asks for forward velocity.

positionError = desiredPosition - actualPosition;

desiredVelocity = Kp_pos * positionError;

end