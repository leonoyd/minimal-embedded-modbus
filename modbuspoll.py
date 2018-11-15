#!/usr/bin/env python
import minimalmodbus

instrument = minimalmodbus.Instrument('/dev/pts/6', 1) # port name, slave address (in decimal)

## Read temperature (PV = ProcessValue) ##
while True:
    temperature = instrument.read_register(289, 1) # Registernumber, number of decimals
    print (temperature)
