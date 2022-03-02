# evaluate_odometry

## Scripts for Odometry Evaluation

This repository provide the scripts for odometry evaluation in SLAM. The descriptions are as follows:

- evaluate.py: the evaluation core;
- evaluation_absolute: absolute pose evalution to get translation error in XYZ, and attitude error in euler angles;
- evaluate_odo: the interface for odometry evaluation;
- gpstime.py: gps time and unix second convertion;
- nav2trj.py: convert latitude-longitude-altitude and euler angle to trajectory file;

**Authors:** Hailiang Tang from the [Integrated and Intelligent Navigation (i2Nav) Group](http://www.i2nav.com/), Wuhan
University.

## 1 Prerequisites

### Please follow the command to install the prerequisites:

```shell
# evo for evaluation
pip install evo

# numpy, matplotlib 
pip install numpy matplotlib

# numpy-quaternion for attitude representation
pip install numpy-quaternion
```

## 2 Acknowledgements

We use [evo](https://github.com/MichaelGrupp/evo) for odometry evaluation.
