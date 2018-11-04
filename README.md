# CleverFlightLib
Library for COEX Clever to simplify and enhance autonomous flight 

# How to use it
Clone or download this repository to Raspberry Pi on your Clever-running copter with PX4
```bash
git clone https://github.com/artem30801/CleverFlightLib
```
Setup your copter using [official COEX docs](https://clever.copterexpress.com/).
Import FlightLib inro your python code for autonomous flight and init ros node with any name
Example:
```python
from FlightLib import FlightLib as drone
FlightLib.init('CleverFlight') 
from FlightLib import LedLib as led
```
