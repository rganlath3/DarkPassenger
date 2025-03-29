# Dark Passenger

[![Hardware License: CERN-OHL-S-2.0](https://img.shields.io/badge/Hardware%20License-CERN--OHL--S--2.0-lightgrey.svg)](https://ohwr.org/cern_ohl_s_v2.txt)
[![Software License: GPL v3](https://img.shields.io/badge/Software%20License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Status: In Development](https://img.shields.io/badge/Status-In%20Development-yellow.svg)]()


A modular payload for use as a ROS learning platform. Each floor of the chassis delivers different capabilities. The initial design is mounted on a roomba.

![Dark Passenger Iso View](/Visual_Documentation/ISOView03192025.png)

<!-- ABOUT THE PROJECT -->
## About The Project
The idea was to reuse a cheap roomba to create a platform for learning ROS and sensor integration. The end goal is to have a project incorporate aluminum extrusion, working with acrylic, ESP32-S3, stepper motors, thermal camera, depth camera, LIDAR, slip rings, DC motor controllers, encoders, 3D printing, INS sensor fusion, relays, GPS, and power distribution.  

### Design Tools & Requirements

#### Software Requirements
- VS Code (Programming IDE)
- Autodesk Fusion360 (Free CAD Software)
- KiCAD 9.0 (Open Source PCB Design Software)
- Ultimaker Cura (Free 3D Slicing Software)

#### Hardware Requirements
- Raspberry Pi 5
- 32GB+ SD Card
- 2x Dewalt 20VDC 2Ah Battery

## Hardware Development

### Bill of Materials
Coming Soon

### Hardware Used
TBD


### Electrical Interconnect Diagram
[Electrical Interconnect Diagram](/Hardware_Development/DrawIO/DarkPassengerElectricalInterconnect.pdf)

### PCBs

#### Power Distribution PCB
<img src="/Visual_Documentation/PowerDistro_ISOView.png" width="800">


This PCB is used to stack 2x DC-DC converters below and a voltage cutoff board on the top. This PCB distributes power from battery inputs to get converted to 12V and 5V. It interfaces with the following components:

- 2x 2Ah Dewalt Batteries
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

### Wiring
TBD

### Power
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
