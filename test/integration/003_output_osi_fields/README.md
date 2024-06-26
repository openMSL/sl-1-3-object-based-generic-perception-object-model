# 003 Output OSI Fields

This test checks if all required OSI fields are filled by the model in the output SensorData.

## System Structure Definition

The system consists of an OSMP binary trace file player, the model itself and a trace file writer.
The trace file player will read the given SensorView trace file as an input for the sensor model.
The output of the sensor model is written to a binary OSI trace file. This trace file is checked by osi-validation in the pipeline.

<img alt="System Structure" src="system_structure.png" width="600">

## Scenario

The scenario contained in the given trace file consists of a vehicle placed on the x-axis (y = 0) in front of the ego vehicle in the sensor's field of view.
The x-coordinates of the objects in this scenario are:

- ego: 10 m
- ego bbcenter2rear: -1.146 m
- object: 25 m

## Metric

osi-validation shall not fail.

## Pass/Fail Criterion

The test fails, if a required field is not filled in the model output SensorData.
