# Dark Passenger

[![Hardware License: CERN-OHL-S-2.0](https://img.shields.io/badge/Hardware%20License-CERN--OHL--S--2.0-lightgrey.svg)](https://ohwr.org/cern_ohl_s_v2.txt)
[![Software License: GPL v3](https://img.shields.io/badge/Software%20License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Status: In Development](https://img.shields.io/badge/Status-In%20Development-yellow.svg)]()


A modular payload for use as a ROS learning platform. Each floor of the chassis delivers different capabilities. The initial design is mounted on a differential drive roomba with plans to expand to treaded or quad-coptor platforms.

![Dark Passenger Iso View](/Visual_Documentation/ISOView03192025.png)

<!-- ABOUT THE PROJECT -->
## About The Project
The idea was to reuse a cheap roomba to create a platform for learning ROS2 and sensor integration. The end goal is to have a project incorporate aluminum extrusion chassis, working with acrylic, ESP32-S3, KiCAD PCB Development, stepper motors, thermal camera, depth camera, LIDAR, slip rings, DC motor controllers, encoders, 3D printing, INS sensor fusion, relays, GPS, SLAM, NAV2 (navigation stack) and power distribution.

### Design Tools & Requirements

#### Software Requirements
- VS Code (Programming IDE)
- Autodesk Fusion360 (Free CAD Software)
- KiCAD 9.0 (Open Source PCB Design Software)
- Ultimaker Cura (Free 3D Slicing Software)

#### Hardware Requirements
- Raspberry Pi 5
- 32GB+ SD Card
- 1x Dewalt 20VDC 2Ah Battery

## Hardware Development

### Bill of Materials
Coming Soon

### Hardware Used
#### Chassis
* 2020 T Slot Aluminum Extrusion
* 2020 2-Way Corner Brackets
* 2020 3-Way Corner Brackets
* M5 T Nuts and Screws
* 8x10 Black Translucent Acrylic Sheets 1/8" thick.

#### Floor 1
* 2x DROK Step Down Voltage Regulator
* 2x 3V Relay Modules
* 1x Custom Power Distribution PCB
* 1x Custom ESP3 GPIO PCB
* 1x ESP32-S3-N8R2
* 1x COB LED Strip Yellow
* 1x COB LED Strip Red
* 1x Cytron Dual Channel 10A DC Motor Driver
* 1x Quick Disconnect 12Pin Connector
* 1x MPU9250 IMU
* 1x GY-NEO6MV2 GPS Module

#### Floor 2
* 2x Acer USB Powered Hub
* 1x Raspberry Pi 5 (SBC)
* 1x Respeaker Microphone Array
* 1x ZED Stereo Labs Depth Camera

#### Floor 3
* 1x A1M8 RPLIDAR LIDAR Sensor
* 1x GPS Antenna

#### Floor 4
* 1x Custom ESP3 GPIO PCB
* 1x ESP32-S3-N8R2
* 1x Taidacent Hollow Slip Ring (Maybe 2x)
* 1x 5mm Flange Coupler
* 2x TMC2209 V1.3 Stepper Motor Driver
* 2x NEMA 17 Stepper (2A, 59Ncm), 48mm
* 1x Chameleon Camera Teledyne CM3-U3-13S2C-CS
* 3x 3V Relay Modules
* 1x MDD3A Dual Channel 3A DC Motor Driver
* 1x NERF Gun Gearbox
* 1x Laser Diode, 5V 1mA
* 1x COB LED Strip Yellow
* 1x 5V or 12V Cooling Fan

#### Misc
* Various Capacitors, Resistors, SMD LEDs, JST Connectors, Wire Crimps, etc.
* Various stranded wire lengths and colors: 16AWG, 22AWG.
* 1x 2.1mm DC Barrel Jacks Pair (Power Input)
* 1x SPST 20A Heavy Duty Toggle Switch (Projector Power and Pi Power)
* 2x SPST 5A Mini Toggle Switch (LED Power)
* 3D Printer: Creality CR-10SProV2

### Internal Block Diagram
<img src="/Visual_Documentation/DarkPassengerInternalBlockDiagram.png" width="800">

### Electrical Interconnect Diagram
[Electrical Interconnect Diagram](/Hardware_Development/DrawIO/DarkPassengerElectricalInterconnect.pdf)
<img src="/Visual_Documentation/DarkPassengerElectricalInterconnectDiagram.png" width="800">



### PCBs
#### Power Distribution PCB
<img src="/Visual_Documentation/PowerDistro_ISOView.png" width="800">

This PCB is used to stack 2x DC-DC converters below and a voltage cutoff board on the top. This PCB distributes power from battery inputs to get converted to 12V and 5V. It interfaces with the following components:

- Up to 2x 2Ah Dewalt Batteries
- Voltage Cutoff CCA (TBD)
- 20VDC to 5VDC Converter (TBD)
- 20VDC to 12VDC Converter (TBD)
- 3x Powered USB Ports
- Spare I2C

#### Base MCU PCB
<img src="/Visual_Documentation/Base_MCU_ISOView.png" width="800">

This PCB is used to interface an ESP32-S3 with the following components:

- GPS Module (GY-NEO6mV2)
- MPU9250 IMU
- Spare I2C
- 2x Toggle Switches
- Neopixel Strip
- 2x Spare GPIOs
- External Reset
- 2x Channel Motor Driver (MDD10A)
- Battery Scaled Input
- Motor Encoders
- FWD LED Relay
- AFT LED Relay
- 5V Distribution

#### Turret MCU PCB
<img src="/Visual_Documentation/Turret_MCU_ISOView.png" width="800">

This PCB is used to interface an ESP32-S3 with the following components:

- GPS Module (GY-NEO6mV2)
- MPU9250 IMU
- Spare I2C
- Neopixel Strip
- External Reset
- 2x Channel Motor Driver (MDD3A)
- 2x NEMA17 Stepper Motors
- 2x TMC2209 Stepper Driver
- Laser Relay
- Turret LED Relay
- Cooling Fan Relay
- 5V Distribution
- 12V Distribution


### Power Budget and Power Draw
TBD

## Software Development
TBD
<!-- GETTING STARTED -->
## Getting Started

TBD
### Prerequisites
TBD

### Installation

TBD

### Common Issues

TBD

## Assets

TBD

<!-- ROADMAP -->
## Roadmap
- [] TBD

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing & Donations

Any donation helps to support parts for future open source projects!
[Buy Me a Coffee ☕](https://www.buymeacoffee.com/rganlath)

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License
The contents of this project itself is licensed under CERN-OHL-S-2.0 and the underlying source code is licensed under GNU GPLv3. Both of these licenses require attribution.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Ranil Ganlath - ranil.ganlath@gmail.com

Project Link: TBD

<p align="right">(<a href="#readme-top">back to top</a>)</p>
