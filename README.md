# neuro-car
HackED 2020 Competition

## Summary

A Muse2 EEG device is used with Fourier transforms to produce a vector of frequencies and amplitudes.
This vector is passed into an SVM to classify brain states, which are used to control an RC car.

We also measure fatigue to issue shocks to a user as a wake-up method.

The SVM is trained to detect the following states, and their corresponding control mappings:

eyes open -> drive forward
eyes closed -> stop driving
eyes left -> turn left ??
eyes right -> turn right ??

fatigue levels as measured by sleep deprivation, quantized over [0, 5]
5 -> shock

## Support Vector Machine

### Data collection

Collect data with Muse2, export to CSV. Each CSV represents a time interval resultant from a Fourier transform.
Empirically, at least 2 minutes of data per state is required for >80% classification accuracy.

Convert the CSV files to the libsvm file format. This will create both eval and train sets.
You can use: `csv_to_libsvm.py ( evalPercentage [0,100] )`
Be sure to set the relevant classes in the Python file's typeDict object.
This will produce .libsvm files. Do not add empty lines to these files as this will affect parsing.

### Training

ThunderSVM is used with C-SVM for multiple class detection: https://github.com/Xtra-Computing/thundersvm/blob/master/docs/index.md
It must be compiled from source for CUDA 10 support: https://github.com/Xtra-Computing/thundersvm/blob/master/docs/get-started.md#installation
Parameters: https://github.com/Xtra-Computing/thundersvm/blob/master/docs/parameters.md

The relevant variables are gamma and C.

Reducing C will reduce the error margin on hyperplane placement during training: https://stats.stackexchange.com/questions/31066/what-is-the-influence-of-c-in-svms-with-linear-kernel
Gamma affects projection from "non-linear" space to linear space. Higher gamma results in smoother non-linear projection.
Additional information on C and gamma: https://www.quora.com/What-are-C-and-gamma-with-regards-to-a-support-vector-machine

These parameters are best found using a grid search.

**Empirically, low gamma (< 0.1) and and high C (> 1) work well with the EEG.**

Training command: `./thundersvm-train -s 0 -t 2 -g 0.01 -c 10 train.libsvm svm.model`

### Prediction

Results in a text file containing the class vector predicted.

Prediction command: `./thundersvm-predict eval.libsvm svm.model pred.vec`

### Full pipeline

`python3 csv_to_libsvm.py 20`
`./thundersvm-train -s 0 -t 2 -g 0.01 -c 10 train.libsvm svm.model`
`./thundersvm-predict eval.libsvm svm.model pred.vec`

## ROS Architecture

ROS melodic is used for a modular architecture. CUDA 10 is required. We use the catkin build system.

### SVM node

Place a trained model in neuro_car/models

`roslaunch neuro_car svm.launch`
Input topic: /muse_filter_data
Outpt topic: /svm/detection

This receives EEG data and broadcasts classifications.

### Car node

`roslaunch neuro_car car.launch`
Input topic: /svm/detection
Output: direct to RC car

This is a simple state machine that checks for stable classifications and issues actions to the RC car.