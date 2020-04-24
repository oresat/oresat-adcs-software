# oresat-adcs-software
Attitude Determination Control System (ADCS). It include GPS, Star Tracker, Magnetorquer, Magnetometer, and Reaction Wheels.


## Dependacies
`python3 python3-pydbus`

## Usage
- `python3 src/main.py` To run as a process
- `python3 src/main.py -d` To run as a daemon
- `python3 src/main.py -h` For help output

## State Machine
![](docs/adcs_statemachine.jpg)


## Class Design
![](docs/adcs_class_design.jpg)

## Class Data Flow
![](docs/adcs_class_data_flow.jpg)

