## Sample data example from GMU-ASL51
This directory contains several sample examples from the GMU-ASL51 dataset. <br>
To access the full dataset, email: ahosain@gmu.edu

### File structures
1. The folder contains one direcotory for each of the 12 subjects in GMU-ASL51. <br>
2. Kinect pose and video data are in the directory ```gmu-asl51-samples``` and openpose poses are in the directory ```openpose_pose_samples```
3. Under each of this subject directory all the sign video samples and pose from that subject are stored. <br>
4. File names are of the following forms,
    1. ```class_subject_sampleid_rgb.avi```; i.e. ```doorbell_subject01_15_rgb.avi``` means 15th sample video from the 1st subject of doorbell ASL sign class
    2. ```class_subject_sampleid_bodyData.txt``` carries similar meaning except this is pose data from kinect, each line in this file contains one frame's pose data for the corresponding sign video
    3. ```openpose_pose_samples/subject01_json/``` contains all the pose json files from openpose for the 1st subject, see the main readme about how to run
