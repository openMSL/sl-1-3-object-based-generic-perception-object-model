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

ego_vehicle.vehicle_attributes.bbcenter_to_rear.x = 1.5
ego_vehicle.vehicle_attributes.bbcenter_to_rear.y = 0.0
ego_vehicle.vehicle_attributes.bbcenter_to_rear.z = -0.5

# Add moving objects
obj1 = sv_ground_truth.moving_object.add()
obj1.id.value = 13
obj1.type = 2
obj1.vehicle_classification.type = 2

obj1.base.dimension.length = 5
obj1.base.dimension.width = 2
obj1.base.dimension.height = 1.5

obj1.vehicle_attributes.ground_clearance = 0.3

obj2 = sv_ground_truth.moving_object.add()
obj2.id.value = 15

obj2.type = 2
obj2.vehicle_classification.type = 2

obj2.base.dimension.length = 5
obj2.base.dimension.width = 2
obj2.base.dimension.height = 1.5

# Generate 10 OSI messages for 9 seconds
for i in range(10):
    # Increment the time
    sensor_view.timestamp.seconds += 1
    sv_ground_truth.timestamp.seconds += 1

    obj1.base.position.x = 30.0
    obj1.base.position.y = 0.0
    obj1.base.position.z = 0.0
    obj1.base.orientation.roll = 0.0
    obj1.base.orientation.pitch = 0.0
    obj1.base.orientation.yaw = 0.0

    obj2.base.position.x = 50.0
    obj2.base.position.y = 0.0
    obj2.base.position.z = 0.0
    obj2.base.orientation.roll = 0.0
    obj2.base.orientation.pitch = 0.0
    obj2.base.orientation.yaw = 0.0

    """Serialize"""
    bytes_buffer = sensor_view.SerializeToString()
    f.write(struct.pack("<L", len(bytes_buffer)) + bytes_buffer)

f.close()

print("Output " + filename)

