Readme
******
This is fork of Reflow-Oven-Controller project which aims to control DIY reflow ovens with minimum required hardware:
- Arduino
- Thermocouple (MAX6675 or MAX31855)
- Solid state relay (zero cross trigerred)
- PC/laptop
With no dependency to more hardware like LCD or switch as you can find in Reflow-Oven-Controller.

All the status output is sent to the serial terminal. Reflow proces is started or stopped by sending 's' character to Arduino over serial interface more hardware like LCD or switch as you can find in Reflow-Oven-Controller.

In my implementation I use small "esperanza" oven, MAX6675 and FOTEK SSR-25DA.
