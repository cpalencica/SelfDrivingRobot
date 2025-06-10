# Code Readme
This readme should explain the contents of the code folder and subfolders
Make it easy for us to navigate this space.

## Reminders
- Describe here any software you have adopted from elsewhere by citation or URL
- Your code should include a header with your team member names and date

## Folders
There are two important folders used in this project. The main folder contains the code for the purple cars and the node folder contains the code for the remote control node server.

## main
The code begins by declaring global variables to manage IP Addresses, waypoints, wasd, and relevant initializations and declarations for the various parts of hardware implemented in the project: LEDC, PWM, connectivity, relevant drivers, etc.

### Configuration and Initialization
- wifi_init_state(): Establishes the ESP32 in Wi-Fi station mode and handles connection events with an access point.
- init_gpio(): Initializes the GPIO pins for motor direction control.
- configure_ADC_sensor(): Configures an ADC sensor (Sharp IR Range Sensor) by setting the ADC width and channel attenuation based on the ADC unit, and then characterizes the ADC to calibrate it using the specified parameters.
- configure_led(): Configures an LED by resetting the specified GPIO pin and setting its direction as a push/pull output.
- example_ledc_init(): Initializes the LEDC PWM timers and channels for controlling left and right motors by configuring the timer and setting up the respective channels with specified parameters.
- control_left() + control_right(): Set motors direction to foward once

### Main Tasks
- udp_client_task(): This task implements a UDP client that continuously sends data to a specified server, receives responses, parses the received data to update the robot's position and heading, and handles socket creation, timeout settings, and error logging.
- pid_task(): This task implements a PID control task that continuously calculates the PID output based on the error, integral, and derivative terms, and then calls the actuate function with the calculated output.
- report_sensor() + detect_collision(): This task continuously samples the ADC sensor, performs multisampling to average the readings, converts the raw ADC readings to voltage, and calculates the final voltage. This final voltage is later used in the detect_collision task by converting the final voltage to distance, turning on an LED and setting a stop flag if the distance is less than 25 cm, and turning off the LED and clearing the stop flag otherwise.
- udp_receive_wasd(): This task implements a UDP server that listens for incoming data from the remote control node server. It processes the first character of the recieved data and toggles remote control mode if the character is 'q' and also stores other characters like "w", "a", "s", "d" to be used later in the car_movement function

### Other Relevant Functions
- set_motor_speed(): This function sets the speed of a motor by adjusting the PWM duty cycle for the specified motor channel (left or right) and updating the duty cycle configuration.
- calculate_heading(): This is an important function that calculates the heading error for a car by determining the angle to a waypoint, adjusting it to the robot's coordinate system, and then computing the desired heading heading error. The desired heading and heading error is later used PID task.
- actuate(): This function adjusts the motor speeds based on the PID output by calculating the necessary adjustments and setting the PWM duty cycles for the motors, while ensuring the duty cycles are within valid ranges and handling stop and remote control conditions.

## Node App
The node app is a simple server that sets up a UDP server to receive control commands and an HTTP server to serve a web-based remote control interface. It listens for keypress events ('w', 'a', 's', 'd', 'q') on the web page and sends corresponding UDP messages to an ESP32 device to control the car remotely.

#### Sources:
- ChatGPT
- Driving TT motors with LEDC: https://github.com/BU-EC444/04-Code-Examples/tree/main/tt-motor-ledc
- Remote control code example: https://github.com/BU-EC444/04-Code-Examples/tree/main/remote-control
- Optitrack UDP exmaple code: https://github.com/BU-EC444/04-Code-Examples/tree/main/optitrack-upd-client
- PID Design Pattern: https://github.com/BU-EC444/01-EBook-F2024/blob/main/docs/design-patterns/docs/dp-pid.md