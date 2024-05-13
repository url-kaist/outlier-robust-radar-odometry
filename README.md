# Outlier-RObust RAdar odometry (ORORA)


<p align="center"><img src=materials/orora_title_video.gif alt="animated" /></p>

## :bookmark_tabs: About ORORA (IRCA'23)

* A robust radar odometry method in urban environments

* Please refer our [**paper**][arXivlink] and [**YouTube**](https://www.youtube.com/watch?v=7ZMPtornIHA) for detailed explanations and experimental results!

* Validated on [MulRan][Mulranlink] dataset. 

[arXivlink]: https://arxiv.org/abs/2303.01876
[Mulranlink]: https://sites.google.com/view/mulran-pr/dataset

---
## Requirement Version & Test Environment

* CMake version > 3.13
* gcc/g++ > 9.0


* In Ubuntu 18.04, we use
    * Eigen 3.3
    * Boost 1.5.8
    * OpenCV 3.3 (or 3.4)

* In Ubuntu 20.04, we use 
    * Eigen 3.3
    * Boost 1.71
    * OpenCV 4.2

## How to Build

Just follow the below command:

```
$ mkdir -p \~/catkin_ws/src/ && cd \~/catkin_ws/src
$ git clone https://github.com/url-kaist/outlier-robust-radar-odometry.git
$ cd ..
$ catkin build orora
```

TMI: Following the philosophy of target-oriented CMake, PMC is automatically installed when you run `catkin build orora`

## How to Run and Evaluate Radar Odometry in MulRan dataset

1. Set file tree of the [MulRan](https://sites.google.com/view/mulran-pr/dataset) dataset as follows:
 

```
${MULRAN_DIR}
_____/KAIST03 
     |___global_pose.csv
     |___/gt (Synchonized GT poses are saved)
         |___... 
     |___polar_oxford_form
         |___1567410201812840928.png
         |___1567410202062436262.png
         |___1567410202312110509.png
         |___...    
_____${OTHER SEQ}
     |...
_____...
   
```

2. Generate synchronized ground truth poses to the radar data as follows:

```
$ rosrun orora mulran_generate_gt ${MULRAN_DIR} ${SEQ1} ${SEQ2}...
// e.g.
$ rosrun orora mulran_generate_gt /media/hyungtaelim/UX960NVMe/mulran KAIST03
```

3. Then, set right `seq_dir` in `launch/run_orora.launch` & run the below command

```
roslaunch orora run_orora.launch seq_dir:=${DATA_DIR}
```

For instance, it can be run as follows:
```
roslaunch orora run_orora.launch seq_dir:="/media/shapelim/UX980/UX960NVMe/mulran-radar/KAIST03"
```

Note, `polar_oxford_form` folder should be placed in the `${DATA_DIR}` directory.

4. Run `script/evaluate_odometry.py` as follows:

```
// E.g.
$ python evaluate_odometry.py -f /media/hyungtaelim/UX960NVMe/mulran/KAIST03/outputs/mulran_ORORA_cen2018_0.6_0.75_0.1_0.15708eval_odom.txt
```


## Acknowledgement

Many thanks to [Keenan Burnett](https://github.com/keenan-burnett) to provide outstanding radar odometry codes! 

Please refer to [Yeti-Radar-Odometry](https://github.com/keenan-burnett/yeti_radar_odometry) for more information

---

## :mailbox: Contact Information
If you have any questions, please do not hesitate to contact us
* [Hyungtae Lim][htlink] :envelope: `shapelim at kaist.ac.kr`

[htlink]: https://github.com/LimHyungTae
