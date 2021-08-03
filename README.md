# VINS-Mono
* For the original VINS-Mono package README, check the **README_original.md file**

# Installation
* Run the ``installation/install.sh`` script.
* Clone and build the [VINS Republisher](https://mrs.felk.cvut.cz/gitlab/visual-localization/vins_republish) package. It transforms the odometry from VINS Mono package for MRS system.

# How to run
## Gazebo simulation with F330 model and fisheye camera
* Run the ``scripts/f330_simulation/start.sh`` script.
* After takeoff, navigate to ``VINS republisher`` tab and press **&#8593;**, then Enter

### Using VINS-Mono in the control feedback
* Navigate to ``VINS help`` tab, press once **&#8593;** and Enter. If successfull, press twice **&#8593;** and Enter again.
* In the ``status`` tab, the Odometry section (Hort, Vert and Head) should display ``VIO``.

### Simulation filter
* Old Gazebo had an issue, when some of the messages had same time stamp which caused a lot of issues. Here is a link to a node on the incoming messages https://mrs.felk.cvut.cz/gitlab/visual-localization/simulation_filter
* Test if still happens in the newer Gazebo version