function motor_thrusts = mixer(T, tau_roll, tau_pitch, tau_yaw, Tmax)
% MIXER
% Converts desired total thrust and body torques into four rotor thrusts.
%
% Inputs:
% T         = desired total thrust [N]
% tau_roll  = desired roll torque [N*m]
% tau_pitch = desired pitch torque [N*m]
% tau_yaw   = desired yaw torque [N*m]
% Tmax      = maximum thrust allowed per rotor [N]
%
% Output:
% motor_thrusts = [M1; M2; M3; M4] thrusts [N]

% TODO: We will derive and add the mixer equations next.

motor_thrusts = zeros(4,1);

end