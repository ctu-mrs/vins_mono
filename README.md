# VINS-Mono
For the original VINS-Mono package README, check the [README_original.md](./Readme_original.md) file.

## Installation
* Run the ``installation/install.sh`` script.
* See [vio_core](https://mrs.felk.cvut.cz/gitlab/visual-localization/vio_core) for installation of related packages and example tmux scripts.

## Changes from the original VINS-Mono repository
* using `/UAV_NAME/` namespace for node names, topic names, and TF frames
* flipped the published TF tree to be compatible with the [MRS system TF tree](https://ctu-mrs.github.io/docs/system/frames_of_reference.html)
* custom configs and launch files

## Simulation filter
* Old Gazebo had an issue, when some of the messages had same time stamp which caused a lot of issues. Here is a link to a node on the incoming messages https://mrs.felk.cvut.cz/gitlab/visual-localization/simulation_filter
* Test if still happens in the newer Gazebo version
