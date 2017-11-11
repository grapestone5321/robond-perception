[//]: # (Image References)

[image1]: ./images/downsampling.png "downsampling"

[image2]: ./images/pass_through.png "pass_through"

[image3]: ./images/extracting.png "extracting"

[image4]: ./images/segmentation.png "segmentation"

[image5]: ./images/confusion_marix_1.png "confusion_marix_1"

[image6]: ./images/object_recognition_1.png "object_recognition_1"

[image7]: ./images/confusion_marix_2.png "confusion_marix_2"

[image8]: ./images/object_recognition_2.png "object_recognition_2"

[image9]: ./images/object_recognition_3.png "object_recognition_3"

[image10]: ./images/object_recognition_4.png "object_recognition_4"

## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

I provide a Writeup that includes all the rubric points and how I addressed each one.

The writeup includes a statement and supporting figures / images that explain how each rubric item was addressed, and specifically where in the code each step was handled. The writeup includes a discussion of what worked, what didn't and how the project implementation could be improved going forward.

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

The pcl_callback() function within RANSAC.py has been filled out to include filtering and RANSAC plane fitting.

- See [RANSAC.py](https://github.com/grapestone5321/robond-perception/blob/master/Exercise-1/RANSAC.py)

The steps to complete this exercise are the following. I add screenshots of output.

   1. Downsample my point cloud by applying a Voxel Grid Filter.

![downsampling][image1]

   2. Apply a Pass Through Filter to isolate the table and objects.

![pass_through][image2]

   3. Perform RANSAC plane fitting to identify the table.

   4. Use the ExtractIndices Filter to create new point clouds containing the table and objects separately.
![extracting][image3]
#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

Steps for cluster segmentation have been added to the pcl_callback() function in segmentation.py.

- See [segmentation.py](https://github.com/grapestone5321/robond-perception/blob/master/sensor_stick/scripts/segmentation.py)

To build my perception pipeline, I must perform following steps. I add the screenshot of output.

   1. Create a python ros node that subscribes to /sensor_stick/point_cloud topic. Use segmentation.py file found under /sensor_stick/scripts/ to get started.

   2. Use your code from Exercise-1 to apply various filters and segment the table using RANSAC.
   3. Create publishers and topics to publish the segmented table and tabletop objects as separate point clouds.

   4. Apply Euclidean clustering on the table-top objects (after table segmentation is successful).

   5. Create a XYZRGB point cloud such that each cluster obtained from the previous step has its own unique color.

   6. Finally publishmy colored cluster cloud on a separate topic.

![segmentation][image4]

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

Both compute_color_histograms() and compute_normal_histograms() functions within features.py in /sensor_stick/src/sensor_stick have been filled out and SVM has been trained using train_svm.py.

- See [features.py](https://github.com/grapestone5321/robond-perception/blob/master/sensor_stick/src/sensor_stick/features.py)

- See [train_svm.py](https://github.com/grapestone5321/robond-perception/blob/master/sensor_stick/scripts/train_svm.py)

I provide a snapshot of my normalized confusion matrix output from train_svm.py.

![confusion_marix_1][image5]

Object recognition steps have been implemented in the pcl_callback() function within capture_features.py. I add the screenshot of output.

![object_recognition_1][image6]

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

![confusion_marix_2][image7]

I add the functionality to my already existing ros node that communicates with my perception pipeline to perform sequential object recognition.

- See [project.py](https://github.com/grapestone5321/robond-perception/blob/master/pr2_robot/scripts/project.py)

I save my PickPlace requests into output_1.yaml, output_2.yaml, and output_3.yaml for each scene respectively.

- See [output_1.yaml](https://github.com/grapestone5321/robond-perception/blob/master/output/output_1.yaml)

- See [output_2.yaml](https://github.com/grapestone5321/robond-perception/blob/master/output/output_2.yaml)

- See [output_3.yaml](https://github.com/grapestone5321/robond-perception/blob/master/output/output_3.yaml)

I add screenshots of output showing label markers in RViz to demonstrate my object recognition success rate in each of the three scenarios. My pipeline correctly identify 100% of objects in test1.world, 100% (5/5) in test2.world and 75% (6/8) in test3.world.

![object_recognition_2][image8]

![object_recognition_3][image9]

![object_recognition_4][image10]

