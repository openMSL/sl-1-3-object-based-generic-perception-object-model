# Ground Truth Objects to .csv Output Strategy

This strategy writes ground truth objects in a csv file.
Path for the file is set via CMakeLists.

## Usage
You need to add the name of the strategy in a new line in the *csv_output_sequence.conf* file in the *src/model/strategies/* folder.

In order for the strategy to be called during simulation, the FMI parameter *switch_for_csv_output* needs to be set to *1* and be passed to the framework or model fmu.

NOTE:
This strategy needs transformation-functions from ../transformation-functions.
