# FP.1 Match 3D objects
Matching 3D objects functionality is implemented in `matchBoundingBoxes` function under camFusion_Student.cpp. 
The following steps were taken:

### 1. Initialize a map to Count Matches
The function starts by declaring an std::map `boundingBoxCount` to count the number of keypoint matches between each pair of bounding boxes from the previous and current frames. It then iterates over all of the matches to identify which bounding boxes in both frames contain the matched keypoints.

### 2. Count Matches for Bounding Box Pairs
For each of the matches, the function determnes which bounding boxes in the current and previous frames contain the matched keypoints. It increments the count in the `boundingBoxCount` matrix for each pair of bounding boxes that contain the corresponding keypoints.

### 3. Determine Best Matches
The function then iterates through the bounding boxes to find the best matching bounding box in the current frame for each bounding box in the previous frame. It does this by finding the maximum count in each row of the matrix, indicating the best match based on the number of keypoint matches. The results are stored in the `bbBestMatches` map.

# FP.2 Compute Lidar based TTC
The `computeTTCLidar` function calculates the Time to Collision (TTC) between two consecutive frames of Lidar data. It starts by determining the time difference `dT` between frames based on the given frame rate. The function then extracts the x-coordinates (driving direction) of the Lidar points from both the previous and current frames, computing their median values instead of using the lowest point, which can sometimes be erroneous. Then, it calculates the TTC using the formula \(\text{TTC} = \frac{\text{medianCurrX} \times dT}{\text{medianPrevX} - \text{avgCurrX}}\), where `CurrX` is the points of x-coordinate of the current frame, and `PrevX` is the points x-coordinate of the previous frame. 

# FP.3 Associate Keypoint Correspondences with Bounding Boxes
The `clusterKptMatchesWithROI` function starts by clustering & associating the keypoint matches. If keypoints are in the bounding box, then keypoints are within the ROI are saved. The outliers are removed by computing the norm distance between the keypoints with a threshold (basically average of keypoints )

# FP.4 Compute TTC with Camera Images
TTC with camera images is implemented in function `computeTTCCamera`. The implementation is same as the one learnt in previous lesson on Collision Detection using Camera. Here, the median distace computation was made to remove outliers

# FP.5 and 6 Performance Evaluation
Lidar TTC values do not have any dependencies on detector/descriptor type. I didn't notice any deviation in Lidar estimates. 

Below table shows the distance computed between current and previous lidar frames. Although Lidar TTC estimates are very consistent, between 3rd to 5th frames TTC increases. One reason is the constant velocity model breaks with changes in distance. Because there is higher reduction in distance from 3rd frame the constance velocity model would not work very well here. Similar results can be seen from frames 11 to 12 then 12 to 13.

| Distance(m) |  TTC(s)  |
| ---------|-------|
|  7.97    | 12.515|
|  7.91    | 12.61
|7.68      |14.09
|7.64      |16.689
|7.58      | 15.74
|7.55      | 12.78
|7.47      | 11.98 
|7.43      | 13.024
|7.39      | 11.17
|7.20      | 12.80
|7.27      | 8.95
|7.19      | 9.964
|7.13      | 9.59
|7.04      | 8.521
|6.83      | 9.515
|6.90      | 9.612
|6.81      | 8.39

Analysis and plots are  made in the table for various detector/descriptor combinations in `results.xslx`. It can be noticed that AKAZE works quite well for Camera based TTC. The combination of SHI Tomasi with BRIEF works very well as well.
Some of the descriptor detector combinations failed mainly because of these reasons:
- The keypoint features detected were too less especially inside the detected bounding box. During matching, it is often noticed that the keypoints of the car in the truck lane are matched and this is an outlier. 