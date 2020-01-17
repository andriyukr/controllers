
# ROS Package for AI-Based Controllers for UAVs

This is a repository for ROS package containing different fuzzy logic controllers for controlling unmanned aerial vehicles.

## Overview

**Keywords:** controller, AI, UAV

**Author: Andriy Sarabakha<br />
Affiliation: [Nanyang Technological University (NTU)](https://www.ntu.edu.sg)<br />
Maintainer: Andriy Sarabakha, andriy001@e.ntu.edu.sg**

This package has been tested under [ROS] Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Publications

#### 

If you use this work in an academic context, please cite the following publications:

* A. Sarabakha, C. Fu, and E. Kayacan, "Intuit Before Tuning: Type-1 and Type-2 Fuzzy Logic Controllers," *Applied Soft Computing*, vol. 81, pp. 105495--105510, Aug. 2019. [Paper](https://doi.org/10.1016/j.asoc.2019.105495), [video](http://tiny.cc/FM-FLC).

```bibtex
@article{SARABAKHA2019105495,
title = "Intuit before tuning: Type-1 and type-2 fuzzy logic controllers",
journal = "Applied Soft Computing",
volume = "81",
pages = "105495",
year = "2019",
issn = "1568-4946",
doi = "https://doi.org/10.1016/j.asoc.2019.105495",
url = "http://www.sciencedirect.com/science/article/pii/S1568494619302650",
author = "Andriy Sarabakha and Changhong Fu and Erdal Kayacan",
keywords = "Type-1 fuzzy logic controllers, Interval type-2 fuzzy logic controllers, Fuzzy mapping, Aerial robotics, Unmanned aerial vehicles",
abstract = "Although a considerable amount of effort has been put in to show that fuzzy logic controllers have exceptional capabilities of dealing with uncertainty, there are still noteworthy concerns, e.g., the design of fuzzy logic controllers is an arduous task due to the lack of closed-form input–output relationships which is a limitation to interpretability of these controllers. The role of design parameters in fuzzy logic controllers, such as position, shape, and height of membership functions, is not straightforward. Motivated by the fact that the availability of an interpretable relationship from input to output will simplify the design procedure of fuzzy logic controllers, the main aims in this work are derive fuzzy mappings for both type-1 and interval type-2 fuzzy logic controllers, analyse them, and eventually benefit from such a nonlinear mapping to design fuzzy logic controllers. Thereafter, simulation and real-time experimental results support the presented theoretical findings."
}
```

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

		sudo apt-get install libeigen3-dev


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd ~/catkin_workspace/src
	git clone https://github.com/andriyukr/controllers.git
	cd ../
	catkin_make


<!--- ## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch ros_package_template ros_package_template.launch

## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the content of this config file

Config file folder/set 2

* **...**

## Launch files

* **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

     Argument set 1

     - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

    Argument set 2

    - **`...`**

* **...**

## Nodes

### ros_package_template

Reads temperature measurements and computed the average.


#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

	The temperature measurements from which the average is computed.


#### Published Topics

...


#### Services

* **`get_average`** ([std_srvs/Trigger])

	Returns information about the current average. For example, you can trigger the computation from the console with

		rosservice call /ros_package_template/get_average


#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

	The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

	The size of the cache.


### NODE_B_NAME

...




## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues). --->


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
