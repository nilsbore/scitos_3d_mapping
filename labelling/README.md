Tools to label metric sweeps
============================


Preparation (optional)
----------------------

1. Export the metric sweeps from mongodo
  
  ``` rosrun semantic_map load_from_mongo /path/where/to/save/ ```

  This will extract all the metric maps contained in the current running instance of mongodb and save them to `/path/where/to/save/`
  
2. Copy sweeps corresponding to one of the waypoints somewhere else:

  ``` rosrun semantic_map_labeller export_waypoint_sweeps /path/original_data/ /path/exported_data/ WayPointXXX```

  This will extract sweeps corresponding to `WayPointXXX` from `/path/original_data/` and save them in `/path/exported_data`. You might want to do this if you want to have separate folders with sweeps corresponding to different waypoints, however doing this or not does not affect the labelling. 

Labelling 
----------------------

At this point I will assume for simplicity that the path where the sweeps to be annotated are located is `/path_to_data/` (it can be any folder on the disk, depending on where your data was saved / copied). I will refer to the waypoint for which you want to annotate sweeps as `WayPointXXX`. 

1. List all the sweeps corresponding to one waypoint 

  ``` rosrun semantic_map_labeller list_waypoint_sweeps /path_to_data/ WayPointXXX ```

  This creates a file `/path_to_data/WayPointXXX.txt` which contains all the sweeps corresponding to `WayPointXXX`. The sweeps listed in this file will be loaded by the semgnetation tool later on (step 3.).

2. Extract depth and rgb images from the intermediate point clouds:

  ``` rosrun semantic_map_labeller create_int_images /path_to_data/ WayPointXXX ```

  After doing this, in the sweep folders you should be able to find the rgb & depth images corresponding to the intermediate point clouds. The segmentation tool loads these images (step 3). 

3. Object segmentation (on RGB & depth images). This is done with (python script):

  ``` rosrun semantic_map_labeller grabcut.py /path_to_data/WayPointXXX.txt start_image_index end_image_index start_sweep_number ```

  _Explanation_: 
  * the `start_image_index` and `end_image_index` allow for the segmentation of a subset of the intermediate images (by default there are 51 intermediate positions, but maybe only a subset is interesting). 
  * the `start_sweep_number` tell the program to start from sweep 20 next and not from the beginning. The sweeps are loaded in the order they are listed in `/path_to_data/WayPointXXX.txt`. Useful when the tool crashes and you don't want to go through all the images to reach where you left off.
   
  Normally I would run the tool with:

  ``` rosrun semantic_map_labeller grabcut.py /path_to_data/WayPointXXX.txt 0 16 0 ```
  
  Once the annotation tool starts, you should see 4 windows (you might have to drag them as they could be on top of each other): RGB image, depth image, object mask and object (the last two should be black to begin with). 
  There are a number of commands supported
  * with the right mouse button, you draw a rectangle around the object you want to segment and annotate.
  * by pressing "n" you run an iteration of the region growing/segmentation/grabcut algorithm. __NOTE:__ you need to draw the rectangle before doing this. After pressing "n", the object mask and object windows should display the current version of the segmented object.
  * to fine tune the selected object, you can press "1" and then with the left mouse button you can draw over regions that belong to the object and should be segmented as part of the object. Press "n" after this to see the result. (you can draw on both RGB and depth images)
  * to fine tune the selected object, you can press "0" and then with the left mouse button you can draw over regions that DO NOT belong to the object and should __NOT__ be segmented as part of the object. Press "n" after this to see the result. (you can draw on both RGB and depth images)
  * to reset the current rectangle and segmentation press "r"
  * to save the current object press "s"
  * to move to the next image press "." Note: after reaching the last image of a sweep, the program will automatically load the first image of the next sweep
  * to move to the previous image press "," Note: after reaching the first image of a sweep, the program will automatically load the first image of the previous sweep.
  * if at any point you want to see what the label of the next object that you will save for a particular sweep is, press "f" - this is useful if you think you forgot an object in an image, you go back to that image and you want to check whether you have saved anything at all or not. You can of course also check this with the file browser. 
  Disclaimer: this is not very user friendly, but from my experience, once you get the hang of it  you can annotate very fast. 

  The way it should work is: 
  * draw rectangle, press "n"
  * press "1" draw over object, press "n"
  * press "0" draw outside object, press "n"
  * if happy, press "s" to save object
  * repeat for other objects in the image, or press "." to move to the next image. 


4. Once the object segmentation step (above) is done, we need to assign labels to objects. This is done through the project: QTLabeller.
  This is a QT project, so open it in QT creator and compile it. Next set the following run arguments:

  ```/path_to_data/ WayPointXXX /path_to_data/labels.txt /path_to_data/display_labels.txt``
  
  _Explanation:_ first create two empty text files `/path_to_data/labels.txt` `/path_to_data/display_labels.txt`.   These will hold the labels you can assign to the objects you have segmented, and you can add labels online once you start the program, so they can be empty to begin with. There are two such files, because you might want one label to be displayed when you assign it, but something else, like its alias to be actually saved (for example, in some room there are 3 chairs, with display labels John's chair, Jack's chair and Maria's chair, but I want the actual labels to be chair1, chair2, chair3 - while annotation 1000+ images, I might forget which one was chair1 and which one was chair2, so it's easier to remember with the display labels).

  Run the program:
  * press "n" to go to the next segmented object.
  * press "p" to go to the previous segmented object. 
  You can see a list of available labels, you can add more if you want (by adding corresponding display labels and "regular" labels). The labels in the list are the display labels, but the object will be saved with the "regular" label. For each segmented object, select the label from the list and assign it, and then move on to the next segmented object. 

  __Note:__ if nothing is displayed but the terminal output shows that some sweeps were found, press "n" and the next image should be displayed. If you see a message that no sweeps were found, check your data path and make sure all the arguments are correct. 


5. Next, generate point clouds from the segmented objects by running:
  
  ``` rosrun semantic_map_labeller pcd_from_label /path_to_data/ WayPointXXX ```

  After this you should have .pcd files corresponding to the images you segmented earlier in the sweep folders.

6. Finally, create .xml files for each segmented and labelled object. These are later on read by the routines that return the labelled objects:

  ``` rosrun semantic_map_labeller create_xml_from_labels /path_to_data/ WayPointXXX ```
  
  _Note:_ this will stop at a folder if the number of labels doesn't match the number of pcd files or the number of images. See terminal output. 

Visualize
------------------
As a check, you can now visualize the objects you have labelled with.

1. Per sweep visualization:

  ``` rosrun metaroom_xml_parser load_labelled_data /path_to_data/ WayPointXXX ```

  This iterates through all the sweep folders in /path/to/ and loads and displays labelled objects from the object xml files create previously. Press "q" to move on to the next observation. 

2. Per object label class visualization: 

``` rosrun semantic_map_labeller view_labelled_objects /path_to_data/ WayPointXXX ``` 

  This iterates through all the labels you saved and displays all objects with the same label at once. Press "q" to move on to the next label.
