# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`



### Dataset #1
> Your algorithm will be run against "sample-laser-radar-measurement-data-1.txt". We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [0.09, 0.09, 0.65, 0.65].

```
Accuracy - RMSE:
 0.083163
0.0894975
 0.609715
 0.590668
```

### Dataset #2
> Your algorithm will be run against "sample-laser-radar-measurement-data-2.txt". We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [0.20, 0.20, 0.55, 0.55].
```
Accuracy - RMSE:
0.190264
0.187443
0.286235
 0.48177

```
