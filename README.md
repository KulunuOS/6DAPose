# 6DAPose

This work presents a pointcloud registration based assembly pose estimation method for robotic assembling. The framework consists of semantic
segmentation of the scene and registering point clouds of local surfaces against target point clouds derived from CAD models in order to detect 6D poses.
The method is demonstrated and evaluated on two synthetic object assembly datasets generated in gazebo simulation environment.

**Download the datasets [here](https://zenodo.org/records/10117869)**

Please use the following citation if you used this repository in your publications:

```
@article{samarawickrama20236d,
  title={6D Assembly Pose Estimation by Point Cloud Registration for Robot Manipulation},
  author={Samarawickrama, Kulunu and Sharma, Gaurang and Angleraud, Alexandre and Pieters, Roel},
  journal={arXiv preprint arXiv:2312.02593},
  year={2023}
}
```

## Requirements

The source code is tested under following specifications

- Ubuntu Linux 20.04 LTS
- Python 3.8 or greater
- ROS Noetic
- Gazebo Classic



Install the required python libraries to your anaconda or virtual environment

```
$ pip install -r requirements.txt
```

## Installation Instructions

1. Create a workspace in your local setup following the directory structure:

```
$ mkdir -p ~/6DAPose/src
$ cd ~/6DAPose/src
$ git clone https://github.com/KulunuOS/6DAPose.git .
```
2. download the dependent repositories using vcs-tool

```
$ vcs import < 6DAPose.repos
```
3. move the echo_tfs.py script to tf2_tools package

```
$ mv echo_tfs.py /geometry2/tf2_tools/scripts/
```

4. Finally, build the workspace 

```
$ source /opt/ros/noetic/setup.bash
$ colcon build
```

## Evaluation

### Download Dataset

Download the [datasets](https://zenodo.org/records/10117869) and copy to directories "/fidget_dataset" or "/Nema17_reducer_dataset"

### Generate the Dataset on locally

Follow the instructions below if you want to generate the dataset locally 

1. Launch the gazebo simulation package
```
# Terminal 1
$ source /opt/ros/noetic/setup.bash
$ cd ~/6DAPose
$ source devel/setup.bash

# to launch Nema17 Planetary reducer dataset:
$ roslaunch data_generation nema17.launch

# to launch fidget dataset:
$ roslaunch data_generation fidget.launch
```

2. Launch the tf2_tools package for coordinate frames transformation info 
```
# Terminal 2
$ source /opt/ros/noetic/setup.bash
$ cd ~/6DAPose
$ source devel/setup.bash
# rosrun tf2_tools echo_tfs.py base_link camera_depth_optical_frame

```
3. Launch the script to generate and save data. set argument -d to define dataset and argument -s to define the assembly steps. 
```
# to generate fidget dataset: do for each assemble steps 1-3 by setting argument -s
$ python3 render_icp_data.py -m bottom_casing left_gear right_gear top_casing -d fidget -s 1

# to generate Nema17 reducer dataset: do for each assemble steps 1-4 by setting argument -s
$ python3 render_icp_data.py -m Nema17 sun_gear housing carrier cover -d Nema17_reducer -s 1

```
4. Generate groundtruth assembly specific relative pose information using the preprocess notebooks [1](https://github.com/KulunuOS/6DAPose/blob/main/Fidget_Ground_truth_preprocess.ipynb) and [2](https://github.com/KulunuOS/6DAPose/blob/main/Nema17_dataset_GT_preprocess.ipynb)


### Run evaluation
```
$ python3 6DAPose.py
```

## Acknowledgement

Project funding was received from Helsinki Institute of Physics’ Technology Programme (project; ROBOT) and European Union’s Horizon 2020 research and innovation programme, grant agreement no. 871252 (METRICS).



