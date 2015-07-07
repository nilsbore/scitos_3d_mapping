*** To extract all the sweeps corresponding to one waypoint and save them at another location run the following:
rosrun semantic_map_labeller export_waypoint_sweeps /path/from/ /path/to/ WayPointXXX 
(WayPointXXX for us should be WayPoint5 - kitchen, WayPoint16 - Nils' room, already done, WayPoint19 - my room). 

After doing this, you should have sweeps in /path/to/ along with a txt file /path/to/WayPointXXX.txt which contains a list of xml files for the sweeps you exported. 

*** Next, we need to extract depth and rgb images from the intermediate point clouds:
rosrun semantic_map_labeller create_int_images /path/to/ WayPointXXX

After doing this, in the sweep folders you should be able to find the rgb & depth images corresponding to the intermediate point clouds. We will annotate the images next. 

*** We are now ready to start the annotation. This is done with (python script):
rosrun semantic_map_labeller grabcut.py /path/to/WayPointXXX.txt start_sweep_number end_sweep_number
Explanation: the start and end sweep number are to allow for the annotation of a subset of the sweeps, or if the tool crashes, to be able to tell it that it should start from sweep 20 next and not from the beginning. The sweeps are loaded in the order they are listed in /path/to/WayPointXXX.txt
Once the annotation tool starts, you should see 4 windows (you might have to drag them as they could be on top of each other): RGB image, depth image, object mask and object (the last two should be black to begin with). 
There are a number of commands supported
- with the right mouse button, you draw a rectangle around the object you want to segment and annotate.
- by pressing "n" you run an iteration of the region growing/segmentation/grabcut algorithm. NOTE: you need to draw the rectangle before doing this. After pressing "n", the object mask and object windows should display the current version of the segmented object.
- to fine tune the selected object, you can press "1" and then with the left mouse button you can draw over regions that belong to the object and should be segmented as part of the object. Press "n" after this to see the result. (you can draw on both RGB and depth images)
- to fine tune the selected object, you can press "0" and then with the left mouse button you can draw over regions that DO NOT belong to the object and should NOT be segmented as part of the object. Press "n" after this to see the result. (you can draw on both RGB and depth images)
- to reset the current rectangle and segmentation press "r"
- to save the current object press "s"
- to move to the next image press "." Note: after reaching the last image of a sweep, the program will automatically load the first image of the next sweep
- to move to the previous image press "," Note: after reaching the first image of a sweep, the program will automatically load the first image of the previous sweep.
- if at any point you want to see what the label of the next object that you will save for a particular sweep is, press "f" - this is useful if you think you forgot an object in an image, you go back to that image and you want to check whether you have saved anything at all or not. You can of course also check this with the file browser. 
Disclaimer: this is not very user friendly, but from my experience, once you get the hang of it (about 10 minutes) you can annotate very fast. 
The way it should work is: 
- draw rectangle, press "n"
- press "1" draw over object, press "n"
- press "0" draw outside object, press "n"
- if happy, press "s" to save object
- repeat for other objects in the image, or press "." to move to the next image. 


*** Once the object segmentation step (above) is done, we need to assign labels to objects. This is done through the project: QTLabeller.
This is a QT project, so open it in QT creator and compile it. Next set the following run arguments:
/path/to/ WayPointXXX /path/to/labels.txt /path/to/display_labels.txt
Explanation: first create two empty text files /path/to/labels.txt /path/to/display_labels.txt. These will hold the labels you can assign to the objects you have segmented, and you can add labels online once you start the program, so they can be empty to begin with. There are two such files, because you might want one label to be displayed when you assign it, but something else, like its alias to be actually saved (for example, in Nils' room there are 3 chairs, with display labels Nils' chair, Johan's chair and Yuquan's chair, but I want the actual labels to be chair1, chair2, chair3, in case I upload the data anywhere I don't want people to see Yuquan's chair there but chair2; on the other hand, while annotation 1000+ images, I might forget which one was chair1 and which one was chair2, so it's easier to remember with the display labels).

Run the program:
- press "n" to go to the next segmented object.
- press "p" to go to the previous segmented object. 
You can see a list of available labels, you can add more if you want (by adding corresponding display labels and "regular" labels). The labels in the list are the display labels, but the object will be saved with the "regular" label. For each segmented object, select the label from the list and assign it, and then move on to the next segmented object. 


*** Next, generate point clouds from the segmented objects by running:
rosrun semantic_map_labeller pcd_from_label /path/to/ WayPointXXX
After this you should have .pcd files corresponding to the images you segmented earlier in the sweep folders.

*** Finally, create .xml files for each segmented and labelled object. These are later on read by the routines that return the labelled objects. 
rosrun semantic_map_labeller create_xml_from_labels /path/to/ WayPointXXX

*** As a check, you can now visualize the objects you have labelled with:
rosrun metaroom_xml_parser load_labelled_data /path/to/ WayPointXXX
This iterates through all the sweep folders in /path/to/ and loads and displays labelled objects from the object xml files create previously.
