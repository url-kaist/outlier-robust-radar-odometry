# Outlier-RObust RAdar odometry (ORORA)

## :bookmark_tabs: About ORORA (IRCA'23)

* A robust radar odometry method in urban environments

<p align="center"><img src=materials/orora_title_video.gif alt="animated" /></p>

**NOTE** Code & youtube video will be updated until the end of Feb!

* Please refer our **paper** for detailed explanantions and experimental results!

* Validated on [MulRan][SemanticKITTIlink] dataset. 

* :bulb: Contents: **YouTube**

[YouTubeLInk]: https://www.youtube.com/watch?v=fogCM159GRk
[arXivlink]: https://arxiv.org/abs/2207.11919
[SemanticKITTIlink]: https://sites.google.com/view/mulran-pr/dataset

---
## Requirement Version & Test Environment

* CMake version > 3.13
* gcc/g++ > 9.0


* In Ubuntu 18.04, we use
    * Eigen 3.3
    * Boost 1.5.8
    * OpenCV 3.3 (or 3.4)

* In Ubuntu 20.04, we use
    * Eigen TBU
    * Boost TBU
    * OpenCV TBU

## How to Build

Based on target-oriented CMake, PMC is automatically installed when you run `catkin build orora`

```asm
$ mkdir -p \~/catkin_ws/src/ && cd \~/catkin_ws/src
$ git clone https://github.com/url-kaist/outlier-robust-radar-odometry.git
$ cd ..
$ catkin build orora
```

## How to Run and Evaluate Radar Odometry in MulRan dataset

1. Set files of the [MulRan](https://sites.google.com/view/mulran-pr/dataset) dataset as follows:
 

```asm
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

```asm
$ roslaunch orora run_orora.launch
```

4. Run `script/evaluate_odometry.py` as follows:

```asm
// E.g.
$ python evaluate_odometry.py -f /media/hyungtaelim/UX960NVMe/mulran/KAIST03/outputs/mulran_ORORA_cen2018_0.6_0.75_0.1_0.15708eval_odom.txt
```


## Acknowledgement

Many thanks to [Keenan Burnett](https://github.com/keenan-burnett/yeti_radar_odometry) to provide some outstanding radar odometry codes!

---

## :mailbox: Contact Information
If you have any questions, please do not hesitate to contact us
* [Hyungtae Lim][htlink] :envelope: `shapelim at kaist.ac.kr`

[htlink]: https://github.com/LimHyungTae
