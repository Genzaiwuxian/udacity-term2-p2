# udacity-term2-p2
Environment: Win7+Docker;

1) Code must compile without errors with cmake and make.
2) RMSE px<0.09, py<0.10, vx<0.40 and vy<0.30
  a) laser + radar sensor: dataset1 & dataset2
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset1.PNG)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset1_NIS.PNG)
  
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p1/blob/master/figure/radar%26laser_dataset2.png
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset1.PNG)
  
  b) laser sensor open: dataset1 & dataset2
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p1/blob/master/figure/laser_dataset1.png)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset1.PNG)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p1/blob/master/figure/laser_dataset2.png)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset1.PNG)
  
  c) radar sensor open: dataset1 & dataset2
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p1/blob/master/figure/radar_dataset1.png)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset1.PNG)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p1/blob/master/figure/radar_dataset2.png)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset1.PNG)

3) Follows the Correct Algorithm
  tools: calculate the jacbian matrix and RMSE;
  Kalmanfilter: prediction and update(for laser)&updateEKF(for radar)
  FusionEKF: initialise like R, H, timesamp, first x_ value, etc. then call prediction, and Update/UpdateKF function based on differnt sensor.

4) remarks:
  a) Laser+Radar is better than sigle sensor;
  b) between laser and radar, laser have good performance than radar in prediction of px and py;
  c) for vx and vy, laser and radar have no big differnce, it seems that radar have better result in vx, and laser better in vy.
