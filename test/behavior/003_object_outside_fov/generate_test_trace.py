# Copyright 2024 BMW AG
# SPDX-License-Identifier: MPL-2.0

from osi3.osi_sensorview_pb2 import SensorView
import struct
import datetime

# Set trace file name
now = datetime.datetime.now()
timestamp = now.strftime("%Y%m%dT%H%M%SZ")
filename = timestamp + "_sv_360_2112_10_test_trace.osi"

f = open(filename, "ab")
# Generate SensorView
sensor_view = SensorView()
sensor_view.version.version_major = 3
sensor_view.version.version_minor = 6
sensor_view.version.version_patch = 0

sensor_view.timestamp.seconds = 0
sensor_view.timestamp.nanos = 0

# Generate GroundTruth inside SensorView
sv_ground_truth = sensor_view.global_ground_truth
sv_ground_truth.version.version_major = 3
sv_ground_truth.version.version_minor = 6
sv_ground_truth.version.version_patch = 0

sv_ground_truth.timestamp.seconds = 0
sv_ground_truth.timestamp.nanos = 0
sv_ground_truth.host_vehicle_id.value = 7

# Add (stationary) ego vehicle
ego_vehicle = sv_ground_truth.moving_object.add()
ego_vehicle.id.value = 7
ego_vehicle.type = 2
ego_vehicle.vehicle_classification.type = 2

ego_vehicle.base.dimension.length = 5
ego_vehicle.base.dimension.width = 2
ego_vehicle.base.dimension.height = 1.5

ego_vehicle.vehicle_attributes.bbcenter_to_rear.x = -1.5
ego_vehicle.vehicle_attributes.bbcenter_to_rear.y = 0.0
ego_vehicle.vehicle_attributes.bbcenter_to_rear.z = -0.5

# Add moving object
moving_object = sv_ground_truth.moving_object.add()
moving_object.id.value = 13

# Generate 10 OSI messages for 9 seconds
for i in range(10):
    # Increment the time
    sensor_view.timestamp.seconds += 1
    sv_ground_truth.timestamp.seconds += 1

    moving_object.type = 2
    moving_object.vehicle_classification.type = 2

    moving_object.base.dimension.length = 5
    moving_object.base.dimension.width = 2
    moving_object.base.dimension.height = 1.5

    moving_object.base.position.x = 400.0
    moving_object.base.position.y = 0.0
    moving_object.base.position.z = 0.0

    moving_object.base.orientation.roll = 0.0
    moving_object.base.orientation.pitch = 0.0
    moving_object.base.orientation.yaw = 0.0

    """Serialize"""
    bytes_buffer = sensor_view.SerializeToString()
    f.write(struct.pack("<L", len(bytes_buffer)) + bytes_buffer)

f.close()

print("Output " + filename)

