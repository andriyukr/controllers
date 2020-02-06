
# ROS Package for AI-Based Controllers for UAVs

This is a repository for ROS package containing different fuzzy logic controllers for controlling unmanned aerial vehicles.

## Overview

**Keywords:** controller, AI, UAV

**Author: Andriy Sarabakha<br />
Affiliation: [Nanyang Technological University (NTU)](https://www.ntu.edu.sg)<br />
Maintainer: Andriy Sarabakha, andriy001@e.ntu.edu.sg**

This package has been tested under [ROS] Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Publications

#### Type-1 Fuzzy Logic Controller

For more details on type-1 fuzzy logic controllers, please refer to: 

* C. Fu, A. Sarabakha, E. Kayacan, C. Wagner, R. John, and J. M. Garibaldi, "Input Uncertainty Sensitivity Enhanced Non-Singleton Fuzzy Logic Controllers for Long-Term Navigation of UAVs," *IEEE/ASME Transactions on Mechatronics*, vol. 23, no. 2, pp. 725--734, Apr. 2018. [Paper](https://doi.org/10.1109/TMECH.2018.2810947), [video](http://tiny.cc/SLAM-FLC).

If you use this work in an academic context, please cite our paper:
```bibtex
@ARTICLE{8304792,  
author={C. Fu and A. Sarabakha and E. Kayacan and C. Wagner and R. John and J. M. Garibaldi},  
journal={IEEE/ASME Transactions on Mechatronics},  
title={Input Uncertainty Sensitivity Enhanced Nonsingleton Fuzzy Logic Controllers for Long-Term Navigation of Quadrotor UAVs},  
year={2018},  
volume={23},  
number={2},  
pages={725-734},  
keywords={aircraft control;autonomous aerial vehicles;C++ language;cameras;control engineering computing;fuzzy control;fuzzy set theory;helicopters;Kalman filters;operating systems (computers);position control;robot vision;sensor fusion;SLAM (robots);three-term control;input uncertainty sensitivity enhanced nonsingleton fuzzy logic controllers;inertial measurement unit;unmanned aerial vehicles;input uncertainties;antecedent fuzzy sets;nonsingleton inference;intersection;antecedent FSs;Cen-NSFLC;Tra-NSFLC;quadrotor UAV;proportional-integral derivative controller;visual-inertial SLAM;control performance;visual-inertial simultaneous localization and mapping;long-term navigation;robot operating system;C++ programming language;GPS-denied unknown environments;Uncertainty;Simultaneous localization and mapping;Frequency selective surfaces;Real-time systems;IEEE transactions;Mechatronics;Fuzzy logic controller (FLC);input uncertainty sensitivity enhanced nonsingleton FLC (NSFLC);monocular visual-inertial simultaneous localization and mapping (SLAM);NSFLC;unmanned aerial vehicle (UAV)},  
doi={10.1109/TMECH.2018.2810947},  
ISSN={1941-014X},  
month={April}
}
```

#### Interval Type-2 Fuzzy Logic Controller

For more details on interval type-2 fuzzy logic controllers, please refer to: 

* E. Kayacan, A. Sarabakha, S. Coupland, R. John, and M. A. Khanesar, "Type-2 Fuzzy Elliptic Membership Functions for Modeling Uncertainty," *Engineering Applications of Artificial Intelligence*, vol. 70, pp. 170--183, Apr. 2018. [Paper](https://doi.org/10.1016/j.engappai.2018.02.004).

If you use this work in an academic context, please cite our paper:
```bibtex
@article{KAYACAN2018170,
title = "Type-2 Fuzzy Elliptic Membership Functions for Modeling Uncertainty",
journal = "Engineering Applications of Artificial Intelligence",
volume = "70",
pages = "170 - 183",
year = "2018",
issn = "0952-1976",
doi = "https://doi.org/10.1016/j.engappai.2018.02.004",
url = "http://www.sciencedirect.com/science/article/pii/S0952197618300253",
author = "Erdal Kayacan and Andriy Sarabakha and Simon Coupland and Robert John and Mojtaba Ahmadieh Khanesar",
keywords = "Elliptic membership function, Type-2 fuzzy logic theory, Uncertainty, Fuzzy sets, Gaussian, Triangular, Addition, Multiplication, Fuzzy arithmetics",
abstract = "Whereas type-1 and type-2 membership functions (MFs) are the core of any fuzzy logic system, there are no performance criteria available to evaluate the goodness or correctness of the fuzzy MFs. In this paper, we make extensive analysis in terms of the capability of type-2 elliptic fuzzy MFs in modeling uncertainty. Having decoupled parameters for its support and width, elliptic MFs are unique amongst existing type-2 fuzzy MFs. In this investigation, the uncertainty distribution along the elliptic MF support is studied, and a detailed analysis is given to compare and contrast its performance with existing type-2 fuzzy MFs. Furthermore, fuzzy arithmetic operations are also investigated, and our finding is that the elliptic MF has similar features to the Gaussian and triangular MFs in addition and multiplication operations. Moreover, we have tested the prediction capability of elliptic MFs using interval type-2 fuzzy logic systems on oil price prediction problem for a data set from 2nd Jan 1985 till 25th April 2016. Throughout the simulation studies, an extreme learning machine is used to train the interval type-2 fuzzy logic system. The prediction results show that, in addition to their various advantages mentioned above, elliptic MFs have comparable prediction results when compared to Gaussian and triangular MFs. Finally, in order to test the performance of fuzzy logic controller with elliptic interval type-2 MFs, extensive real-time experiments are conducted for the 3D trajectory tracking problem of a quadrotor. We believe that the results of this study will open the doors to elliptic MFs’ wider use of real-world identification and control applications as the proposed MF is easy to interpret in addition to its unique features."
}
```

#### Fuzzy Mapping-Based Controller

For more details on fuzzy mapping-based controllers, please refer to: 

* A. Sarabakha, C. Fu, and E. Kayacan, "Intuit Before Tuning: Type-1 and Type-2 Fuzzy Logic Controllers," *Applied Soft Computing*, vol. 81, pp. 105495--105510, Aug. 2019. [Paper](https://doi.org/10.1016/j.asoc.2019.105495), [video](http://tiny.cc/FM-FLC).

If you use this work in an academic context, please cite our paper:
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

#### Artificial Neural Network-Based Controller

For more details on artificial neural network-based controllers, please refer to: 

* S. Patel, A. Sarabakha, D. Kircali, and E. Kayacan, "An Intelligent Hybrid Artificial Neural Network-Based Approach for Control of Aerial Robots," *Journal of Intelligent & Robotic Systems*, pp. 1--12, May 2019. [Paper](https://doi.org/10.1007/s10846-019-01031-z), [video](http://tiny.cc/failure\_ANN).

If you use this work in an academic context, please cite our paper:
```bibtex
@Article{Patel2019,
author="Patel, Siddharth
and Sarabakha, Andriy
and Kircali, Dogan
and Kayacan, Erdal",
title="An Intelligent Hybrid Artificial Neural Network-Based Approach for Control of Aerial Robots",
journal="Journal of Intelligent {\&} Robotic Systems",
year="2019",
month="May",
day="04",
abstract="In this work, a learning model-free control method is proposed for accurate trajectory tracking and safe landing of unmanned aerial vehicles (UAVs). A realistic scenario is considered where the UAV commutes between stations at high-speeds, experiences a single motor failure while surveying an area, and thus requires to land safely at a designated secure location. The proposed challenge is viewed solely as a control problem. A hybrid control architecture -- an artificial neural network (ANN)-assisted proportional-derivative controller -- is able to learn the system dynamics online and compensate for the error generated during different phases of the considered scenario: fast and agile flight, motor failure, and safe landing. Firstly, it deals with unmodelled dynamics and operational uncertainties and demonstrates superior performance compared to a conventional proportional-integral-derivative controller during fast and agile flight. Secondly, it behaves as a fault-tolerant controller for a single motor failure case in a coaxial hexacopter thanks to its proposed sliding mode control theory-based learning architecture. Lastly, it yields reliable performance for a safe landing at a secure location in case of an emergency condition. The tuning of weights is not required as the structure of the ANN controller starts to learn online, each time it is initialised, even when the scenario changes -- thus, making it completely model-free. Moreover, the simplicity of the neural network-based controller allows for the implementation on a low-cost low-power onboard computer. Overall, the real-time experiments show that the proposed controller outperforms the conventional controller.",
issn="1573-0409",
doi="10.1007/s10846-019-01031-z",
url="https://doi.org/10.1007/s10846-019-01031-z"
}
```

#### Fuzzy Neural Network-Based Controller

For more details on fuzzy neural network-based controllers, please refer to: 

* A. Sarabakha, N. Imanberdiyev, E. Kayacan, M. A. Khanesar, and H. Hagras, "Novel Levenberg–Marquardt–Based Learning Algorithm for Unmanned Aerial Vehicles," *Information Sciences*, vol. 417, pp. 361--380, Nov. 2017. [Paper](https://doi.org/10.1016/j.ins.2017.07.020), [video](http://tiny.cc/FNN).

If you use this work in an academic context, please cite our paper:
```bibtex
@article{SARABAKHA2017361,
title = "Novel Levenberg–Marquardt Based Learning Algorithm for Unmanned Aerial Vehicles",
journal = "Information Sciences",
volume = "417",
pages = "361 - 380",
year = "2017",
issn = "0020-0255",
doi = "https://doi.org/10.1016/j.ins.2017.07.020",
url = "http://www.sciencedirect.com/science/article/pii/S0020025517308393",
author = "Andriy Sarabakha and Nursultan Imanberdiyev and Erdal Kayacan and Mojtaba Ahmadieh Khanesar and Hani Hagras",
keywords = "Fuzzy neural networks, Sliding mode control, Levenberg–Marquardt algorithm, Type-1 fuzzy logic control, Unmanned aerial vehicle",
abstract = "In this paper, Levenberg–Marquardt inspired sliding mode control theory based adaptation laws are proposed to train an intelligent fuzzy neural network controller for a quadrotor aircraft. The proposed controller is used to control and stabilize a quadrotor unmanned aerial vehicle in the presence of periodic wind gust. A proportional-derivative controller is firstly introduced based on which fuzzy neural network is able to learn the quadrotor’s control model on-line. The proposed design allows handling uncertainties and lack of modelling at a computationally inexpensive cost. The parameter update rules of the learning algorithms are derived based on a Levenberg–Marquardt inspired approach, and the proof of the stability of two proposed control laws are verified by using the Lyapunov stability theory. In order to evaluate the performance of the proposed controllers extensive simulations and real-time experiments are conducted. The 3D trajectory tracking problem for a quadrotor is considered in the presence of time-varying wind conditions."
}
```

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

		sudo apt-get install libeigen3-dev
		
- [MAVROS](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation) (communication node for ROS)

		sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras

		wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
		./install_geographiclib_datasets.sh

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd ~/catkin_ws/src
	git clone https://github.com/andriyukr/controllers.git
	cd ..
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
