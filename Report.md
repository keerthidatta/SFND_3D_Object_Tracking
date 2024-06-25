#FP.1 Match 3D objects
Matching 3D objects functionality is implemented in `matchBoundingBoxes` function under camFusion_Student.cpp. 
The following steps were taken:

### 1. Initialize a 2D Vector to Count Matches
The function starts by initializing a 2D vector `boundingBoxCount` to count the number of keypoint matches between each pair of bounding boxes from the previous and current frames. It then iterates over all of the matches to identify which bounding boxes in both frames contain the matched keypoints.

### 2. Count Matches for Bounding Box Pairs
For each of the matches, the function determnes which bounding boxes in the current and previous frames contain the matched keypoints. It increments the count in the `boundingBoxCount` matrix for each pair of bounding boxes that contain the corresponding keypoints.

### 3. Determine Best Matches
The function then iterates through the `boundingBoxCount` matrix to find the best matching bounding box in the current frame for each bounding box in the previous frame. It does this by finding the maximum count in each row of the matrix, indicating the best match based on the number of keypoint matches. The results are stored in the `bbBestMatches` map.

#FP.2 Compute Lidar based TTC
The `computeTTCLidar` function calculates the Time to Collision (TTC) between two consecutive frames of Lidar data. It starts by determining the time difference `dT` between frames based on the given frame rate. The function then extracts the x-coordinates (driving direction) of the Lidar points from both the previous and current frames, computing their average values instead of using the lowest point, which can sometimes be erroneous. Then, it calculates the TTC using the formula \(\text{TTC} = \frac{\text{avgCurrX} \times dT}{\text{avgPrevX} - \text{avgCurrX}}\), where `avgCurrX` is the average x-coordinate of the current frame, and `avgPrevX` is the average x-coordinate of the previous frame. 

#FP.3 Associate Keypoint Correspondences with Bounding Boxes
The `clusterKptMatchesWithROI` function starts by clustering & associating the keypoint matches. If keypoints are in the bounding box, then keypoints are within the ROI are saved. The outliers are removed by computing the norm distance between the keypoints with a threshold (basically average of keypoints scaled with a small constant)

#FP.4 Compute TTC with Camera Images
TTC with camera images is implemented in function `computeTTCCamera`. The implementation is same as the one learnt in previous lesson on Collision Detection using Camera. Here, the median distace computation was made to remove outliers

#FP.5 and 6 Performance Evaluation
Lidar TTC values do not have any dependencies on detector/descriptor type. I didn't notice any large deviation in Lidar estimates.
Analysis is made in the table for various detector/descriptor combinations in `results.csv`. It can be noticed that AKAZE works quite well for Camera based TTC.