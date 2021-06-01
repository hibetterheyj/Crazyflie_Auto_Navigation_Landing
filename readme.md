# CrazyPracticals (21 Spring)

> **Members: Yujie He, [Jianhao Zheng](https://github.com/Jianhao-zheng/), and [Longlai Qiu](https://github.com/kevinxqiu)**
>
> **[[Video](https://youtu.be/RP4-SlhOIUk)] [[Code](https://github.com/hibetterheyj/Crazyflie_Auto_Navigation_Landing/tree/master/code/crazyflie-lib-python/group_7)] [[Slide](https://drive.google.com/file/d/1vY_UMflVXOcUSOASHkGHsSTXCBmwrVhK/preview)]**

## Goal: Autonomous Navigation and Landing for Crazyflie

In this practical, we programed based on [Crazyflie  2.1](https://www.bitcraze.io/products/crazyflie-2-1/) to find and precisely land on a platform with height of 10 cm by utilizing z reading from [flow deck](https://www.bitcraze.io/products/flow-deck-v2/). Additionally, We also utilized sensor readings from [multi-ranger deck](https://www.bitcraze.io/products/multi-ranger-deck/) to avoid the obstacles presented in the environment.

<p align="center"><img src="./pics/cover.jpg" alt="cover" width="800"/></p>


## Pipeline

| Autonomous navigation & landing                              | Workflow                                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ✓ Local obstacle avoidance<br />✓ Grid-based coverage path planning<br />✓ Waypoint following<br />✓ A* search-based re-planning | <img src="./pics/pipeline_final.png" alt="pipeline_final"  width="500"/> |



## Code

> Code folder: `./code/crazyflie-lib-python/group_7/`

```
.
├── cf_load_params.py
├── cf_search.py
├── cf_state_class.py
├── cf_utilis.py
├── overall.py
├── draw_traj_demo.py
├── logs
│   ├── overall-20210530_1930_x.csv
│   ├── overall-20210530_1930_x_half.csv
│   ├── overall-20210530_1930_y.csv
│   └── overall-20210530_1930_y_half.cs
└── readme.md
```

- `overall.py`: overall pipeline from taking off to landing.

  ```shell
  # -x (float) for setting initial x position
  # -y (float) for setting initial y position
  # -v (bool) for enabling visualization
  python overall.py -x 0.6 -y 0.6 -v
  ```

  <p align="center"><img src="./pics/cf_land.gif" alt="cf_land" width="600"/></p>

- `draw_traj.py`: x-y trajectory visualization with region annotation

  ```shell
  # --log_folder (str) for assigning input log folder
  # --logname (str) for loding log file
  # --img_folder (str) for assigning output image folder
  # -n/--name (str) for assigning output image name
  # --zone_anno (bool) for enabling region annotation
  python draw_traj_demo.py --logname overall-20210530_1930 -n cf_demo --zone_anno
  ```

  <p align="center"><img src="./pics/cf_demo.png" alt="cf_demo"   width="600" /></p>
The estimated values drift considerably after long flights. Moreover, the predicted starting position is significantly different from the starting point after the drone re-takes off.



## Experimental setup

| Features                                                     | Figures                                                      |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ✓ Size: 480 cm (W) × 120 cm (H)<br/>✓ Starting & Landing pad<br/>  - starting (x, y) = (60 cm, 60 cm)<br/>  - landing pad randomly placed<br/>✓ Circular and rectangular obstacles | <img src="./pics/experimental_setup.png" alt="experimental_setup"  width="500" /> |



## Features

- Modular library for different tasks

  ```
  ├── cf_load_params.py  # parameter setting
  ├── cf_search.py       # searching functions such as, coverage planning, box edge detection, A* search
  ├── cf_state_class.py  # state estimation class for the proposed task
  └── cf_utilis.py       # utility functions, such as live plotting
  ```

- Utilized `argparse` for quick parameter adjustment and tuning

- Utilized `matplotlib` for real-time visualization



## Acknowledgement

Thanks to Prof. Dario Floreano and TAs from LIS at EPFL for these amazing tutorials and examples!
