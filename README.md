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
To install this package, simply run `make all` in the top level directory. Run
`make help` for information about the other available installation commands.

[42]:https://github.com/ericstoneking/42
