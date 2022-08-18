# Robots_movements
control robots movements in real world

BOM:

Microcontroller: NodeMCU (esp8266)

Motors and the controllers can be found here: https://www.aliexpress.com/item/4000021207668.html?spm=a2g0o.order_list.0.0.6b941802aDl5fe
Motors: BLDC Geared Motor with PWM speed control and feedback signal
Motor-controllers: 2 x Dual H-Bridge pwm speed controller
These motors have 5 wires:
1. Red wire: + DC power supply, 
2. Black wire: GND
3. Yellow wire: FG signal line outputs signal, the motor rotates one turn and output six pulse signals
4. Blue wire: PWM pulse width speed control line, can recieve to 0 To 5V
5. White wire: is the line that controls the motor to rotate clockwise or counterclockwise. 

This design is for a symmetric squared shape robot
You have to measure the size of the wheels and the distance between them and fill in all the parameters.
This robot interacts with ROS with the following nodes:
  Subscriber:
    /cmd_vel : geometry messages
    /debugme : recieve 's' and then it start to self-calibrate speed and movements
  Publisher:
    /chatter : data (speed / connection etc`)
    /vel_mux : geometry feedback in real world (todo)
    
Most importent - credits and appreciation - this code is using the wonderful work of:
  ROS Serial Arduino library - Joshua Frank @frankjoshua77 https://github.com/frankjoshua/rosserial_arduino_lib
  LinearRegression for arduino - Javier Alcubierre Villasol @cubiwan https://github.com/cubiwan/LinearRegressino
  Tachometer - Alex Gyver @AlexGyver https://github.com/GyverLibs/Tachometer
