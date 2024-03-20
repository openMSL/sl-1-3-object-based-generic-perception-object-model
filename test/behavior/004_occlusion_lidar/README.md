# 004 Occlusion Lidar

This test checks if an object occluded by another object is not detected by the lidar model.

## System Structure Definition

The system consists of an OSMP binary trace file player, the model itself and a trace file writer.
The trace file player will read the given SensorView trace file as an input for the sensor model.
The output of the sensor model is written into a binary SensorData trace file for the subsequent analysis.

<img alt="System Structure" src="system_structure.png" width="600">

## Scenario

The scenario contained in the generated trace file consists of two object vehicles placed on the x-axis (y = 0) in front of the ego vehicle in the sensor's field of view.
The x-coordinates of the objects in this scenario are:

- ego: 0 m
- ego bbcenter2rear: -1.5 m
- object 1: 30 m
- object 2: 50 m

## Metric

Object 2 is expected to be occluded and therefore not seen by the sensor
In the analyze.py script the average number of detected moving objects over all simulation time steps is calculated.

## Pass/Fail Criterion

The test fails, if it differs from the expectation value of 1.0, as only one vehicle is expected to be seen.
