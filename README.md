# OreSat ADCS Software
## Sub-projects
- *ADCS library* - A common library for the ADCS Manager and ADCS Simulator.
to test.
- *ADCS Manager* - A daemon that manages the ADCS on OreSat. 
- *ADCS Simulator* - A program that simulates the ADCS on OreSat for 
software-in-the-loop testing. 
- *ADCS 42 Interface* - A dbus to socket data converter allowing [42] to 
display a simulation of what the satellite is doing.

## Package setup
First, install the following dependencies:
`sudo apt install python3-stdeb python-all dh-python`
Then, in the folder containing `setup.py`, run:
`python3 setup.py --command-packages=stdeb.command bdist_deb`
This will produce a .deb file for this package.

[42]:https://github.com/ericstoneking/42
