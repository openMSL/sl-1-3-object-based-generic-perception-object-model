# Object Based Generic Perception Object Model

![SPDX workflow](https://github.com/openMSL/object_based_generic_perception_object_model/actions/workflows/spdx.yml/badge.svg)

<img align="right" src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/object-based-generic-perception-object-model/uploads/17c84e9ec0acf0fac2e35855f038ad0b/fzdlogo.jpg" width="100" />

This model is a highly parameterizable generic perception sensor and tracking model.
It can be parameterized as a Lidar or a Radar.
The model is based on object lists and all modeling is performed on object level.
It includes typical sensor artifacts like soft FoV transitions, different detection ranges for different targets, occlusion effects depending on the sensor technology as well as simulation of tracking behavior.
The model output are object lists for OSI SenorData moving objects.

<img src="https://user-images.githubusercontent.com/27010086/148824525-3e8e2ff6-9075-4814-a889-caa9ec2d37bf.gif" width="800" />

The architecture of the model as well as the parameterization structure are designed to be as generic as possible to fit both radar and lidar sensors to utilize similarities in signal propagation and signal processing in both technologies.
This way, the model can be parameterized to model different kinds of lidar and radar sensors.
To give an example: You can set an irradiation pattern for the modeled sensor.
Depending on the sensor technology this can either be an antenna gain pattern for radar or a beam pattern for lidar.

## Modeling Approach

### Modeling Framework

The outer layer of the model is the  [Modular OSMP Framework](https://gitlab.com/tuda-fzd/perception-sensor-modeling/modular-osmp-framework) by FZD.
It specifies ways in which models using the [Open Simulation Interface (OSI)](https://github.com/OpenSimulationInterface/open-simulation-interface) are to be packaged for their use in simulation environments using [FMI 2.0](https://fmi-standard.org/).

The actual logic of the model is packed in a so called strategy.
This is where the magic happens.
The `apply()` function of the strategy is called by the `do_calc()` function of the Framework.
The strategy itself is structured into four modules as shown in the image below.

<img src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/object-based-generic-perception-object-model/uploads/5d2135b178a1c22a1ef37b61950df689/model_overview.png" width="300" />

### Sensor Technology Independent Modeling

The **first module** in the figure above brings the received ground truth stationary and moving objects (potentially also traffic signs and traffic lights from sensor_view.global_ground_truth) into a common format.
This enables iterations over all objects regardless of classification.
Then they are transformed to the sensor coordinate system for the following calculations.

In the **last module**, the tracked objects are transformed to the virtual sensor coordinate system (here: vehicle coordinate system) and the fields in the sensor data requested by the HADf are filled.

### Modeling of Specific Sensor Effects

Requirement analysis revealed mainly three characteristic sensor effects of automotive lidars and radars:
(Partial) occlusion, restricted angular fov, and limited detection range due to the ![r^4](https://latex.codecogs.com/svg.latex?&space;r^4) law for signal attenuation and different object sizes.
All effects are additionally subject to noise behavior of the sensor data affecting the detected object states as well as the object detection.
Via functional decomposition, the object-based model is structured to cover these effects with respect to the data processing chain.
As shown in the image above, the first block of the model is the front-end for the simulation of the transmission, propagation and reception.
It receives the object bounding boxes and their poses in world coordinates from the simulation tool, so the first step is to transform the object poses to the sensor coordinate frame.

Then, refined bounding box shapes are generated from the ground truth bounding boxes resulting in a list of characteristic vertices per object.
With these vertices, an occlusion calculation is performed, resulting in the visible 3D vertices of the ground truth objects.
In the detection sensing module, a power equivalent value for every object is calculated.
From the functional decomposition, this would usually be located in the front-end as well, but as thresholding is directly part of the power calculation, it is situated within the data extraction block.
This calculation is based on externally defined parameters and contains angular fov and range limits by considering irradiation characteristics, visible surfaces, and the ![r^4](https://latex.codecogs.com/svg.latex?&space;r^4) law.

If the power is above a threshold, visible vertices are transformed into the vehicle frame and along with the power calculations passed to the tracking block.
There, poses and dimensions from calculated vertices and ground truth are estimated and a tracking logic produces the overall model output.

In the following, a more detailed look at the modules is provided:
As mentioned, the first considered effect is object occlusion.
To calculate the occlusion, a projection and clipping method is used:
Every vertex of the refined bounding boxes of the objects is sorted by distance.
Then, for every object all vertices are projected onto a cylinder at unit distance from the sensor and a concave hull is calculated by the concaveman algorithm<sup>[1](#Agafonkin2016)</sup>, based on<sup>[2](#Park2013)</sup>.
Consequently, the visible vertices of all objects with their respective outer hull are determined.
To calculate the visible area of the polygon for each object, they are sorted by distance and clipped with the Vatti clipping algorithm<sup>[3](#Vatti1992)</sup>.
The resulting new vertex points for the hull are reprojected from the 2D cylinder plane onto their original object bounding box to get the desired vertices in 3D space.

The mentioned refined bounding boxes reflect typical vehicle shapes instead of simple cuboids by considering the shape of the rear and the engine cover.
The figure below shows different examples of refined bounding boxes as a side-view. They can be further adapted by adding more vertices to enhance their fidelity, e.g. for the pillar positions, wheels, etc.

<img src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/object-based-generic-perception-object-model/uploads/66db4e3ec90fc7bcdebb040fb2a722e2/refined_bounding_boxes.png" width="300" />

The bounding box shapes are selected depending on the ground truth object class and adjusted to the object's dimensions.
Consequently, this approach reveals a gap in the OSI definition of vehicle classes.
The classification types in OSI refer more to market segments than to the actual vehicle shapes.
This should be improved in upcoming releases of the interface standard.
Therefore, a separation between sedan and station wagon cannot be accomplished due to the same classification as medium car for vehicles with a length between 4.5 m and 5 m.

To introduce the effect of range - and in case of radar range - measurement noise, the positions of the vertices are altered in range and/or angle by applying parameterizable Gaussian noise.
For lidar simulation, the size of the range-rectified projected area ![A_\text{p}](https://latex.codecogs.com/svg.latex?&space;A_\text{p}) of the object calculated with the noisy bounding box vertices is used to calculate a reflection power equivalent value

![\Large P_\text{eq}=\dfrac{\lambda^2G(\phi,\vartheta)A_\text{p}}{r^4}](https://latex.codecogs.com/svg.latex?\Large&space;P_\text{eq}=\dfrac{\lambda^2G(\phi,\vartheta)A_\text{p}}{r^4}) 

For radar, instead of the projected area, a typical radar cross-section ![\sigma}](https://latex.codecogs.com/svg.latex?&space;\sigma) is set, depending on the classification of the object.
The calculation also includes the irradiation gain at the angle to the bounding box center ![G(\phi,\vartheta)}](https://latex.codecogs.com/svg.latex?&space;G(\phi,\vartheta)) as an approximation, the sensor wave length ![\lambda}](https://latex.codecogs.com/svg.latex?&space;\lambda) and the distance to the object r.
The irradiation characteristics in terms of lidar sensors describe the beam pattern and in terms of radar the antenna diagram.
They are defined in the model profile as a two-dimensional matrix of azimuth and elevation angles.
In this matrix, a value between 0 and 1 is specified for each pair of angles, which specifies the irradiated power of the sensor standardized to the maximum in the angle combination.
For lidars, this especially comes into play when they have a flat optical cover, where the irradiated power is typically less on the edges of the fov.
The detection power threshold is determined by the maximum detection range ![r_{\text{max}}](https://latex.codecogs.com/svg.latex?&space;r_{\text{max}}) of a mid-sized vehicle at boresight with an RCS of 10 sqm., specified in the model profile.
With the typical projection area of the mid-sized reference vehicle ![A_\text{p,ref}](https://latex.codecogs.com/svg.latex?&space;A_\text{p,ref}), or typical RCS of the mid-sized reference vehicle ![\sigma_\text{p,ref}](https://latex.codecogs.com/svg.latex?&space;\sigma_\text{p,ref}), the threshold computes to

![\Large \bar{P}_{\text{thres}}=\dfrac{\lambda^2A_\text{p,ref}}{r_{\text{max}}^4}](https://latex.codecogs.com/svg.latex?\Large&space;\bar{P}_{\text{thres}}=\dfrac{\lambda^2A_\text{p,ref}}{r_{\text{max}}^4})

Detection ranges to standard targets can typically be found in data sheets of commercial radar or lidar sensors.
In most cases, a detection probability for the maximum range is given.
Therefore, Gaussian noise is applied to the threshold with a specified standard deviation ![\xi}](https://latex.codecogs.com/svg.latex?&space;\xi), set in the model profile, to 

![\Large P_{\text{thres}}\propto\mathcal{N}(\bar{P}_{\text{thres}},\xi^2)](https://latex.codecogs.com/svg.latex?\Large&space;P_{\text{thres}}\propto\mathcal{N}(\bar{P}_{\text{thres}},\xi^2))

This noisy power equivalent threshold simulates the thresholding typically performed in the signal processing of radar and lidar sensors.
If ![P_\text{eq}}](https://latex.codecogs.com/svg.latex?&space;P_\text{eq}) is greater or equal ![P_{\text{thres}}](https://latex.codecogs.com/svg.latex?&space;P_{\text{thres}}), the object is considered to be detected and the beforehand calculated visible 3D bounding box vertices are passed to the tracking block.

The vertices are passed to the tracking module as LogicalDetections, so a regular tracking algorithm can be applied on them.
Additionally, at a closer look, the vertices reflect the extrema of a lidar point cloud while leaving out the points in between and regular tracking algorithms only select those anyways for object dimension computation.
So for tracking, the inner points do not matter and therefore, the object based approach completely suffices in this case.
From the vertices at first position, orientation, and dimensions are estimated as the detected object state.
The detected objects are compared to a list of previously detected and tracked objects and thereby added, updated or possibly deleted.
The track logic can be specified in the model profile.
It specifies e.g. the number of consecutive cycles an object has to be detected to create a new track or how many cycles an object is tracked without any detection.
Therefore, objects that are hidden now, but where previously detected, are still tracked estimating their movement by either using ground truth information or by predictions based on detected objects of the previous time steps.
The estimation of the object dimensions is also considered and filtered over several cycles.
If an object is no longer detected over a defined number of cycles, it is deleted from the list.
Consideration of the class uncertainties is provided by the model architecture, as well.
The output of the tracking module is a list of tracked objects.

<a name="Agafonkin2016">1</a>: V. Agafonkin. (2016) Conacveman. [Online]. Available: https://github.com/mapbox/concaveman

<a name="Park2013">2</a>: J.-S.  Park  and  S.-J.  Oh,  “A  new  concave  hull  algorithm  and  concaveness measure for n-dimensional datasets,”Journal of Information Science and Engineering, vol. 29, no. 2, p. 19, 2013.

<a name="Vatti1992">3</a>: B. R. Vatti, “A generic solution to polygon clipping,”Communications of the ACM, vol. 35, no. 7, pp. 56–63, 1992.

## Parameterization

The profiles are parameterized in the files `profile_*.hpp.in`.
The parameters are defined in the files `profile.hpp.in`.
The profiles can be extended by the strategies with additional parameters and values in their respective folders as in e.g. `src/model/strategies/tracking-strategy/` with `profile_struct.hpp.in` with the parameters and `profile_*.hpp.in` with the values.

The profile to be loaded for simulation is set via a model parameter defined in the `modelDescription.xml` of the FMU.
The first name in `src/model/profiles/profile_list.conf` is taken as default.
If you would like to have a different one or if your simulation master does not support the configuration of model parameters, you have to adapt the *start* value of the parameter `profile` in `src/osmp/modelDescription.in.xml`.

### Sensor Parameters

| Parameter                            | Description                                                  |
| ------------------------------------ | ------------------------------------------------------------ |
| sensor_type                          | Lidar: 0; Radar: 1                                           |
| sensor_view_configuration            | Update cycle, field of view, physical mounting position w.r.t. center of rear axle, emitter frequency etc. |
| radar_multipath_min_ground_clearance | Minimum ground clearance of an object for a radar to be able to "look underneath" |
| simulate_sensor_failure              | if set to 1, sensor will fail after the time set in "stop_detection_time" to simulate sensor failure |
| stop_detection_time                  | Time in seconds for the sensor to stop detecting objects     |
| fov_azimuth_border_stddev            | Standard deviation of the normal distribution of the angle (in rad) of the fov. Used for object dimension cropping at the edges |
| fov_elevation_border_stddev          | Standard deviation of the normal distribution of the angle (in rad) of the fov. Used for object dimension cropping at the edges |
| vertex_angle_stddev                  | Standard deviation of the normal distribution of the angle (in rad) of detected vertices of bounding boxes |
| vertex_distance_stddev               | Standard deviation of the normal distribution of the distance (in  m) of detected vertices of bounding boxes |

### Data Extraction Parameters

| Parameter              | Description                                                  |
| ---------------------- | ------------------------------------------------------------ |
| reference_range_in_m   | Detection range for mid-size vehicle (RCS = 10 dBsm) at boreside with a detection probability of 50 % |
| max_range_in_m         | Maximum detection range due to physical limits or ambiguity regions |
| irradiation_pattern    | Beam pattern for lidar and antenna characteristics for radar as elevation-azimuth-map with normalized values between 0 and 1 |
| detection_thes_dB_stdv | standard deviation for the detection threshold combined with the noise floor |

### Object Tracking Parameters

| Parameter                                    | Description                                                  |
| -------------------------------------------- | ------------------------------------------------------------ |
| classification_flag                          | 0 = from ground truth; 1 = all "Unknown Big"                 |
| orientation_flag                             | 0 = from ground truth; 1 = from current point cloud segment  |
| dimension_and_position_flag                  | 0 = from ground truth;<br/>1 = from current point cloud segment;<br/>2 = dimension from current point cloud segments with lower bounds, position as center of manipulated pcl segment;<br/>3 = maximum dimension of current and mean of historical point cloud segments, position as center of manipulated pcl segment;<br/>4 = maximum dimension of current and mean of historical point cloud segments with lower bounds, position as center of manipulated pcl segment; |
| minimum_object_dimension                     | Minimum dimension in m for detected objects                  |
| historical_limit_dimension                   | Limits the historical data used for historical mean dimension calculation |
| velocity_flag                                | 0 = from ground truth; 1 = derivation of position            |
| tracking_flag                                | 0 = ideal (track all ground truth objects); 1 = realistic lidar tracking behavior |
| existence_probability_threshold_for_tracking | Threshold for existence probability, tracking is enabled above threshold |
| min_detections_in_segment_for_tracking       | Minimum no. of detections per segment to track it            |
| existence_probability_increment              | Increment for existence probability                          |
| existence_probability_decrement              | Decrement for existence probability                          |

## Configuration

### Model name

The model's name (in this case "ObjectBasedLidarObjectModel") used for CMake-projects and the FMU at the end is defined in file `model_name.conf` located at `src/model`.

### Install path

When building and installing, the framework will build an FMU package into `FMU_INSTALL_DIR`, which can be used with a simulation tool that supports OSI and fills the required fields listed below.

### VariableNamingConvention

The parameter variableNamingConvention for the FMU specified within the modelDescription.xml is taken from file `variableNamingConvention.conf` located at `src/osmp`.
Possible values are "flat" or "structured".

## Inferface

### Required SensorViewConfiguration (parameterized in profile_*.hpp.in) to be Set in the Simulation Tool

- For every simulated physical sensor system:
  - sensor_view_configuration.mounting_position.position
  - sensor_view_configuration.mounting_position.orientation
  - sensor_view_configuration.update_cycle_time
  - sensor_view_configuration.range
  - sensor_view_configuration.field_of_view_horizontal
  - sensor_view_configuration.field_of_view_vertical

### Required Fields in OSI3 Sensor_View Filled at the Input by the Simulation Tool

- sensor_view.mounting_position
- sensor_view.global_ground_truth.timestamp
- sensor_view.global_ground_truth.host_vehicle_id
- sensor_view.global_ground_truth.stationary_object.id
- sensor_view.global_ground_truth.stationary_object.base.position
- sensor_view.global_ground_truth.stationary_object.base.orientation
- sensor_view.global_ground_truth.stationary_object.base.dimension
- sensor_view.global_ground_truth.stationary_object.classification.type
- sensor_view.global_ground_truth.moving_object.id
- sensor_view.global_ground_truth.moving_object.base.position
- sensor_view.global_ground_truth.moving_object.base.orientation
- sensor_view.global_ground_truth.moving_object.base.orientation_rate
- sensor_view.global_ground_truth.moving_object.base.velocity
- sensor_view.global_ground_truth.moving_object.base.dimension
- sensor_view.global_ground_truth.moving_object.type
- sensor_view.global_ground_truth.moving_object.vehicle_classification.type
- sensor_view.global_ground_truth.moving_object.vehicle_attributes.bbcenter_to_rear
- sensor_view.global_ground_truth.moving_object.vehicle_attributes.ground_clearance

### Additionally Filled Fields in OSI3 Sensor_Data by the Sensor Model

---

**NOTE**

Currently, all information on model input is passed to the output.

---

- sensor_data.timestamp
- sensor_data.moving_object_header.measurement_time
- sensor_data.moving_object_header.cycle_counter
- sensor_data.moving_object_header.data_qualifier
- sensor_data.moving_object.header.ground_truth_id
- sensor_data.moving_object.header.tracking_id
- sensor_data.moving_object.header.age
- sensor_data.moving_object.base.position
- sensor_data.moving_object.base.orientation
- sensor_data.moving_object.base.orientation_rate
- sensor_data.moving_object.base.velocity
- sensor_data.moving_object.base.acceleration
- sensor_data.moving_object.base.dimension
- sensor_data.moving_object.reference_point
- sensor_data.moving_object.movement_state
- sensor_data.moving_object.candidate.probability
- sensor_data.moving_object.candidate.type

## Build Instructions in Windows 10

### Install Dependencies in Windows 10

1. Install cmake from https://github.com/Kitware/CMake/releases/download/v3.20.3/cmake-3.20.3-windows-x86_64.msi
2. Install protobuf for [MSYS-2020](install_protobuf_Win64_MSYS-2020.md) or [Visual Studio 2017](install_protobuf_Win64_VS2017.md)

### Clone with SSH incl. Submodules, Build, and Install in Windows 10

1. Clone this repository [with SSH](https://docs.gitlab.com/ee/ssh/README.html) <ins>incl. submodules</ins>:
   ```bash
   $ git clone git@gitlab.com:tuda-fzd/perception-sensor-modeling/object-based-generic-perception-object-model.git --recurse-submodules
   ```
2. Build the model in [MSYS-2020](install_model_Win64_MSYS-2020.md) or [Visual Studio 2017](install_model_Win64_VS2017.md)
3. Take FMU from `FMU_INSTALL_DIR`

    (Please note that sources are not packed into the FMU at the moment.)

## Build Instructions in Ubuntu 18.04 / 20.04

### Install Dependencies in Ubuntu 18.04 / 20.04

1. Install cmake 3.12
   * as told in [these install instructions](install_cmake_ubuntu_3-12.md)
2. Install protobuf 3.0.0:
   * Check your version via `protoc --version`. It should output: `libprotoc 3.0.0`
   * If needed, you can install it via `sudo apt-get install libprotobuf-dev protobuf-compiler`
   * or from source:
     * Download it from https://github.com/protocolbuffers/protobuf/releases/tag/v3.0.0 and extract the archive.
     * Try to run `./autogen.sh`, if it fails, download the gmock-1.7.0.zip from https://pkgs.fedoraproject.org/repo/pkgs/gmock/gmock-1.7.0.zip/073b984d8798ea1594f5e44d85b20d66/gmock-1.7.0.zip, extract it into the protobuf folder and rename the gmock-1.7.0 folder to gmock.
     * Proceed with the install with
     ```bash
     $ make
     $ sudo make install
     $ sudo ldconfig # refresh shared library cache.
     ```

### Clone with SSH incl. Submodules, Build, and Install in Ubuntu 18.04 / 20.04

1. Clone this repository [with SSH](https://docs.gitlab.com/ee/ssh/README.html) <ins>incl. submodules</ins>:
    ```bash
    $ git clone git@gitlab.com:tuda-fzd/perception-sensor-modeling/object-based-generic-perception-object-model.git --recurse-submodules
    ```
2. Build the model by executing in the extracted project root directory:
    ```bash
    $ mkdir cmake-build
    $ cd cmake-build
    # If FMU_INSTALL_DIR is not set, CMAKE_BINARY_DIR is used
    $ cmake -DCMAKE_BUILD_TYPE=Release -DFMU_INSTALL_DIR:PATH=/tmp ..
    $ make -j N_JOBS
    ```
3. Take FMU from `FMU_INSTALL_DIR`

    (Please note that sources are not packed into the FMU at the moment.)

## Licensing

**Please read file [COPYING](COPYING), which is located in the project root, carefully.**

## Credits

C. Linnhoff, P. Rosenberger, and H. Winner, [*“Refining Object-Based Lidar Sensor Modeling — Challenging Ray Tracing as the Magic Bullet,”*](https://ieeexplore.ieee.org/document/9548071)  IEEE Sensors Journal, Volume 21, Issue 21, Nov. 1, 2021

If you find our work useful in your research, please consider citing: 

```
@ARTICLE{linnhoff2021,
  author={Linnhoff, Clemens and Rosenberger, Philipp and Winner, Hermann},
  journal={IEEE Sensors Journal}, 
  title={Refining Object-Based Lidar Sensor Modeling — Challenging Ray Tracing as the Magic Bullet}, 
  year={2021},
  volume={21},
  number={21},
  pages={24238-24245},
  doi={10.1109/JSEN.2021.3115589}
}
```

This work received funding from the research project 
"[SET Level](https://setlevel.de/)" of the [PEGASUS ](https://pegasus-family.de) project family, promoted by the German Federal Ministry for Economic Affairs and Energy based on a decision of the German Bundestag.
| SET Level                                                                                                | PEGASUS Family                                                                                                       | BMWi                                                                                                                                                                                 |
|----------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| <a href="https://setlevel.de"><img src="https://setlevel.de/assets/logo-setlevel.svg" width="100" /></a> | <a href="https://pegasus-family.de"><img src="https://setlevel.de/assets/logo-pegasus-family.svg" width="100" /></a> | <a href="https://www.bmwi.de/Redaktion/DE/Textsammlungen/Technologie/fahrzeug-und-systemtechnologien.html"><img src="https://setlevel.de/assets/logo-bmwi-en.svg" width="100" /></a> |

We would like to thank Yifei Jiao for his contribution to the first prototype.

Thanks also to all contributors of the following libraries:

- [Open Simulation Interface](https://github.com/OpenSimulationInterface/open-simulation-interface), a generic interface based on protocol buffers for the environmental perception of automated driving functions in virtual scenarios
- [FMI Version 2.0: FMI for Model Exchange and Co-Simulation](https://fmi-standard.org/downloads/)
- [Eigen](http://eigen.tuxfamily.org/), a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
- [Concaveman](https://github.com/sadaszewski/concaveman-cpp), an algorithm for concave hull generation
- [Clipper](http://www.angusj.com/delphi/clipper.php), an open-source polygon clipping algorithm (Boost Software License V1.0)
