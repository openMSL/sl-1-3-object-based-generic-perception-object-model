# Regression Test

Related to [#7](https://github.com/openMSL/sl-1-3-object-based-generic-perception-object-model/issues/7).

This test checks if the virtual sensor mounting position set in the SensorView input is used correctly by the model.

One object is placed in front of  the sensor. The x-coordinates of the objects in this scenario are:
- ego: 10 m
- ego bbcenter2rear: -1.146 m
- virtual sensor mounting position: 0.5 m
- object 25 m

Therefore, the object in virtual sensor coordinates is expected to be located around x = 15.646 m.