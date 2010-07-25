$Id: README.TripleSens,v 1.1 2010/5/15 22:18:07 ckharnett Exp $

README for TripleSensor Board
@author ckharnett <c0harn01@louisville.edu>

Description:

This code was developed at U of Louisville for the SALAMANDER sediment tracker.
The TripleSensor boards use DS18S20 thermometer and DS2450 A/D converter ICs.

v1.1: The code will also detect a DS2438 battery monitor and read the voltage and temperature at the wireless node.
This can be used to track the input voltage of voltage-divider based sensors (the flow sensor and photocell, for example); note that the sensed voltage
will become inaccurate if the supply voltage dips below 2.4V

All the above DS ICs use the 1-Wire protocol, so the original DS2411 code from UCB was expanded here to include
SearchROM and MatchROM functions as well as convert data from the temperature and A/D channels.

This module should go in the folder
c:/cygwin/opt/tinyos-1.x/tos/platform/telosb.
The following files are needed in the folder, and have all been named to not 
interfere with existing telosb files in the original Moteiv installation.
    TripleSens.nc
    TripleSensM.nc  --Main code to modify if you changed the sensorboard wiring
    TripleSensC.nc

    SensorChainPin.nc  --these set the pin that will be used to communicate with the sensor boards (1-Wire data pin)
    SensorChainPinC.nc                
    SensorChainPinM.nc

    SensorChainOn.nc --these switch the sensor chain on and off, currently pin not connected
    SensorChainOnC.nc  --but could be used to save power by shutting off 1-W chips
    SensorChainOnM.nc

    SequencerOscOn.nc  --these switch on an oscillator, currently pin not connected
    SequencerOscOnM.nc  --but oscillator can be used to determine sensor sequence if a frequency counter is implemented.
    SequencerOscOnC.nc

TripleSens defines the interface to the board. SensorChainPin defines the
sensor hardware IO pin.  SensorChainOn and SequencerOscOn can use A/D pins to save power and detect the spatial layout of the sensors, 
but currently these outputs are not connected to anything on the basic power boards.

An application called Salamander uses this module to discover and poll the sensor boards.
This application goes in
c:/cygwin/opt/moteiv/tinyos-1.x/contrib/uofl/Salamander.


   



