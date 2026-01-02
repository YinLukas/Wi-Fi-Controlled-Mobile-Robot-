# Wi-Fi-Controlled-Mobile-Robot-
This project implements a four-wheel mecanum mobile robot with both manual and autonomous control modes. The platform is driven by DC motors through H-bridge drivers and measured using Hall-effect encoders. An ESP32-S3 microcontroller performs real-time closed-loop velocity control using PID + feed-forward PWM, enabling smooth and stable motion at low and high speeds. For manual operation, the robot exposes a browser-based teleoperation interface. The ESP32-S3 runs in Wi-Fi AP mode, allowing any device to connect directly and control the robot without additional networking hardware. For autonomous behavior, the robot uses Time-of-Flight distance sensors to detect obstacles and perform navigation, avoidance, and path tracking. Together, the hardware and software form a full stack mechatronic control system.
## CAD Design
![CAD Model](CAD%20design.jpg)

## Final Robot
![Final Robot](Final%20robot.jpg)
