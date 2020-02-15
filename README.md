# Installation Instructions

```bash
#Instal Pre-Reqs
sudo apt-get remove cmdtest
sudo apt-get install tmuxinator yarn

#Install Ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-melodic-ros-base
sudo sh -c 'echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc'
source ~/.bashrc

#Install NodeJS
curl -sL https://deb.nodesource.com/setup_13.x | sudo -E bash -
sudo apt-get install -y nodejs

#Install Yarn
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
sudo apt-get update
sudo apt-get install yarn

#Build ThunderSVM
cd thundersvm
mkdir build && cd build && cmake .. && make -j

#Build Neuro-Car
cd catkin_ws
catkin_make

#Source NeuroCar Bashrc
echo "source ~/neuro-car/scripts/neuro_car_bashrc.sh" >> ~/.bashrc'
source ~/.bashrc
```

You can ignore the middle right window, that exists for hosting on a web server versus locally.


# YeetMind 🧠

[![MIT license](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://github.com/wagonhelm/neuro-car/blob/master/LICENSE)

HackED 2020 Competition

## Web Functionality

Open:

[To select Muse2 bluetooth device](http://localhost:3000/)

[For accessing locally hosted webserver](http://localhost:43343/)

[For accessing live hosted webserver](http://neurotransmitter.tech/)

## Summary :book:

A Muse2 EEG device is used with Fourier transforms to produce a vector of frequencies and amplitudes.
This vector is passed into an SVM to classify brain states, which are used to control an RC car.

We also measure fatigue to issue shocks to a user as a wake-up method.

The SVM is trained to detect the following states, and their corresponding control mappings:

+ eyes open :arrow_right: drive forward
+ eyes closed :arrow_right: stop driving
+ eyes left :arrow_right: turn left ??
+ eyes right :arrow_right: turn right ??

fatigue levels as measured by sleep deprivation, quantized over [0, 5]
5 -> shock

## Support Vector Machine

### Data collection :1234:

Collect data with Muse2, export to CSV. Each CSV represents a time interval resultant from a Fourier transform.
Empirically, at least 2 minutes of data per state is required for >80% classification accuracy.

Convert the CSV files to the libsvm file format. This will create both eval and train sets.
You can use: `csv_to_libsvm.py ( evalPercentage [0,100] )`
Be sure to set the relevant classes in the Python file's typeDict object.
This will produce .libsvm files. Do not add empty lines to these files as this will affect parsing.

### Training

[ThunderSVM](https://github.com/Xtra-Computing/thundersvm/blob/master/docs/index.md) is used with C-SVM for multiple class detection.
It must be [compiled from source](https://github.com/Xtra-Computing/thundersvm/blob/master/docs/get-started.md#installation) for CUDA 10 support.
[These are the parameters.](https://github.com/Xtra-Computing/thundersvm/blob/master/docs/parameters.md)

The relevant variables are gamma and C.

Reducing C will reduce the error margin on hyperplane placement during training: [Stack Exchange](https://stats.stackexchange.com/questions/31066/what-is-the-influence-of-c-in-svms-with-linear-kernel)
Gamma affects projection from "non-linear" space to linear space. Higher gamma results in smoother non-linear projection.
Additional information on C and gamma: [Quora](https://www.quora.com/What-are-C-and-gamma-with-regards-to-a-support-vector-machine)

These parameters are best found using a grid search.

**Empirically, low gamma (< 0.1) and and high C (> 1) work well with the EEG.**

Training command: `./thundersvm-train -s 0 -t 2 -g 0.01 -c 10 train.libsvm svm.model`

### Prediction

Results in a text file containing the class vector predicted.

Prediction command: `./thundersvm-predict eval.libsvm svm.model pred.vec`

### Full pipeline

`python3 csv_to_libsvm.py 20`
`./thundersvm-train -s 0 -t 2 -g 0.01 -c 10 -m 2048 train.libsvm svm.model`
`./thundersvm-predict eval.libsvm svm.model pred.vec`

## ROS Architecture

ROS melodic is used for a modular architecture. CUDA 10 is required. We use the catkin build system.

### SVM node

Place a trained model in neuro_car/models

`roslaunch neuro_car svm.launch`
Input topic: /muse_filter_data
Outpt topic: /svm/detection

This receives EEG data and broadcasts classifications.

### Car node :car:

`roslaunch neuro_car car.launch`
Input topic: /svm/detection
Output: direct to RC car

This is a simple state machine that checks for stable classifications and issues actions to the RC car.
