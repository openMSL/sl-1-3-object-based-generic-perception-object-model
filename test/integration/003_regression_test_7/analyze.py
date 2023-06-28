# Copyright 2023 BMW AG
# SPDX-License-Identifier: MPL-2.0

import sys
from lib.osi_trace import OSITrace


def main():
    if len(sys.argv) == 2:
        trace_file_name = str(sys.argv[1])
        print("Analyzing " + trace_file_name)
    else:
        print("Usage: python3 analyze <trace_file>")
        exit(1)

    # Load trace file
    trace_file = OSITrace(0)
    trace_file.from_file(trace_file_name, "SensorData")

    # Parameter to check
    avg_det_obj_pos_x = 0
    # Expectation value
    exp_det_obj_pos_x = 15.646
    exp_tolerance = 0.1

    # Iterate over all time steps
    valid_time_steps = 0
    for current_time_step in range(0, trace_file.timestep_count):
        current_message = trace_file.get_message_by_index(current_time_step)
        current_sensor_data = current_message.value.to_dict()
        sensor_view = current_sensor_data["sensorView"][0]

        # Calculate parameter to check
        if "movingObject" in current_sensor_data:
            moving_objects = current_sensor_data["movingObject"]
            if len(moving_objects) == 1:
                current_moving_object = moving_objects[0]
                position = current_moving_object["base"]["position"]
                avg_det_obj_pos_x = avg_det_obj_pos_x + position["x"]
                valid_time_steps = valid_time_steps + 1

    # Calculate parameter to check
    avg_det_obj_pos_x = avg_det_obj_pos_x / valid_time_steps

    trace_file.trace_file.close()

    # Check parameter against expectation value
    if avg_det_obj_pos_x < exp_det_obj_pos_x - exp_tolerance or avg_det_obj_pos_x > exp_det_obj_pos_x + exp_tolerance:
        print("::error title=TraceFileAnalysis::Average x position of detected moving objects is {0:.3f}, "
              "expected value is {1:.3f}".format(avg_det_obj_pos_x, exp_det_obj_pos_x))
        exit(1)

    print("Average x position of detected moving objects {0:.3f} "
          "is within the tolerance around expected value {1:.3f}".format(avg_det_obj_pos_x, exp_det_obj_pos_x))


if __name__ == "__main__":
    main()
