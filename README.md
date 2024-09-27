# OreSat ADCS Software

Python package to assist with the Attitude Determination and Conrol System (ADCS) of OreSat satellites.



### Getting Started

Please use pip to install from github.

`pip install git+https://github.com/oresat/oresat-adcs-software.git` (or some variation)

The package is named `oresat_adcs` so for your python script, import the subpackages you want

For example
```
from oresat_adcs.function import vector
```

Some object instances require loading a json configuration. See `examples`.



### Submodules

##### `oresat_adcs.functions`

Each module in the `functions` subpackage hold a collection of functions to use

- `vector`: For vector operations
- `quaternion`: For quaternion operations
- `frame`: For coordinate transformations
- `attitude_estimation`: For attitude estimation algorithms


##### `oresat_adcs.configuration`

Each module contains classes for customizable objects. Some of these may import `oresat_adcs.functions`

- `structure`: For satellite harware (Satellite walls, magnetorquers, reaction wheels, etc.)
- `environment`: For orbital parameters (drag, gravity, etc.) at a specific moment and location
- `state_machine`: For state machine handling of missions and mission transitions 


##### `oresat_adcs.classes`

Predefined classes

These typically depend on both `oresat_adcs.functions` and `oresta_adcs.configuration`

- `jday`: (Julian Day) Keep track of time
- `sensors`: Represent common sensors
- `dynamic`: Handles dynamics from the environment
- `observer`: Observer for control theory
- `controller`: Controller for control theory 


##### `oresat_adcs.system`

- `simulator`: Simulates an orbit for testing ADCS controllers
- `manager`: Combines controllers and observers into a complete feedback system
- `manager_class`: Combines controllers and observers into a complete feedback system



### Contributing

Join Portland State Aerospace Society! [https://www.pdxaerospace.org/](https://www.pdxaerospace.org/)

Read about OreSat! [https://www.oresat.org/](https://www.oresat.org/)
