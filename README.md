# V2V_Communication Vehical to Vehical Communication
The Dramatic increase in the traffic flow raises demand on innovative technologies that can improve
safety and efficiency of transportation systems. Road safety can be substantially enhanced by the
deployment of wireless communication technologies for vehicular networks, which enable new services
such as collision detection traffic management, and further communication facilities between moving
vehicles. Aiming at providing reliable wireless communications for vehicular networks the wireless
communication will serve as an underlying protocol for future inter-vehicular applications worldwide.
This project presents an implementation of a complete vehicle to vehicle communication, designed
according to the specification. In addition to this a cooperative collision system for protection against
misshappening like vehicle collisions that causes loss of human lives is being implemented. The blind
spot detection system will be useful while changing the lane. Ultrasonic sensors, ARM Cortex –M4
based STM32 Bit Microcontroller, Wifi Module ESP8266,I2C LCD are used to implement the complete
design.

ESP8266 NodeMCU Configuration:
Transmitter Configuration:
1. Setup Network SSID.
2. IP address assignment DHCP.
3. Setup Soft AP(Access Point) P-2-P.
4. Setup network protocol(UDP or TCP/IP).
5. Serial reading from Rx pin connected to tx pin of STM32 board.
6. If data is avaliable start UDP Transmission.

Receiver Configuration:
1. Search avaliable network within range and connect.
2. Read the data and store it in a buffer.
3. Transmit reaceived data to LCD Display.
