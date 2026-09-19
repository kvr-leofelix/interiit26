Position controller: "I want to be at x = 5 m." → tells velocity controller "move forward at 1 m/s."
Velocity controller: "I want 1 m/s." → tells attitude controller "tilt forward."
Attitude controller: "I want this tilt." → tells rate controller "rotate this quickly."
Rate controller → produces torque.
--------------------------
position error
V
position_controller.m
V
desired velocity
V
velocity_controller.m
V
desired pitch
V
attitude_controller.m
V
desired pitch rate
V
rate controller
V
torque