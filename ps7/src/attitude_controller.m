function desiredRate = attitude_controller(desiredAngle, actualAngle, Kp_att)
% ATTITUDE_CONTROLLER
%
% Converts desired attitude angle into desired angular rate.
%
% Input:
% desiredAngle [roll; pitch; yaw] rad
% actualAngle  [roll; pitch; yaw] rad
%
% Output:
% desiredRate [p; q; r] rad/s

angleError = desiredAngle - actualAngle;

desiredRate = Kp_att .* angleError;

end