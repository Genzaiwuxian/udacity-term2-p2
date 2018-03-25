# udacity-term2-p2
Environment: Win7+Docker;

1) Code must compile without errors with cmake and make.
2) RMSE px<0.09, py<0.10, vx<0.40 and vy<0.30
  a) laser + radar sensor: dataset1 & dataset2
  dataset1:
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset1.PNG)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset1_NIS.PNG)
  only 4.0% NIS_Linda exceed 5.991, and 4.9792% NIS_Radar exceed 7.815
  
  dataset2:
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset2.PNG)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar%26laser_dateset2_NIS.PNG)
  
  b) Radar sensor open: dataset1 & dataset2
  dataset1:
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar_dataset1.PNG)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar_dateset1_NIS.PNG)
  
  dataset2:
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar_dataset2.PNG)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/radar_dateset2_NIS.PNG)
  
  c) Lindar sensor open: dataset1 & dataset2
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/laser_dataset2.PNG)
  ![Image text](https://github.com/Genzaiwuxian/udacity-term2-p2/blob/master/figure/laser_dateset2_NIS.PNG)

3) Follows the Correct Algorithm
  tools: calculate the jacbian matrix and RMSE;
  UKF: prediction and update(for laser)&updateEKF(for radar), initialise like std_a, std_yaw, weights, etc.

4) remarks:
  a) Laser+Radar is better than sigle sensor;
  b) UKF show good performance on vx and vy prediction;
