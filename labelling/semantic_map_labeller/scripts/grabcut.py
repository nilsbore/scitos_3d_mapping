#!/usr/bin/env python
'''
===============================================================================
Interactive Image Segmentation using GrabCut algorithm.

This sample shows interactive image segmentation using grabcut algorithm.

USAGE:
    python grabcut.py <filename>

README FIRST:
    Two windows will show up, one for input and one for output.

    At first, in input window, draw a rectangle around the object using
mouse right button. Then press 'n' to segment the object (once or a few times)
For any finer touch-ups, you can press any of the keys below and draw lines on
the areas you want. Then again press 'n' for updating the output.

Key '0' - To select areas of sure background
Key '1' - To select areas of sure foreground
Key '2' - To select areas of probable background
Key '3' - To select areas of probable foreground

Key 'n' - To update the segmentation
Key 'r' - To reset the setup
Key 's' - To save the results
===============================================================================
'''

import numpy as np
import cv2
import sys

BLUE = [255,0,0]        # rectangle color
RED = [0,0,255]         # PR BG
GREEN = [0,255,0]       # PR FG
BLACK = [0,0,0]         # sure BG
WHITE = [255,255,255]   # sure FG

DRAW_BG = {'color' : BLACK, 'val' : 0}
DRAW_FG = {'color' : WHITE, 'val' : 1}
DRAW_PR_FG = {'color' : GREEN, 'val' : 3}
DRAW_PR_BG = {'color' : RED, 'val' : 2}

# setting up flags
rect = (0,0,1,1)
drawing = False         # flag for drawing curves
rectangle = False       # flag for drawing rect
rect_over = False       # flag to check if rect drawn
rect_or_mask = 100      # flag for selecting rect or mask mode
value = DRAW_FG         # drawing initialized to FG
thickness = 3           # brush thickness

start_image_index = 0
end_image_index = 17
image_counter= 0
label_number = 0
folder_counter = 0
sweeps = []
start_sweep = 0
current_sweep = start_sweep

def find_sweeps(sweep_file):
    files = []
    with open(sweep_file) as f:
        files = f.readlines()
        return files

def find_images_depth(xml_path):
    print 'Finding depth images in: ',xml_path
    import os.path
    index = xml_path.rfind('/')
    folder = xml_path[0:index+1]
    image_list = []

    file_found = True
    index = start_image_index
    while file_found:
        f_index='';
        f_index=str(index);
        f_index = f_index.zfill(4);
        f_path = folder+'depth_'+f_index+'.png'
        if os.path.isfile(f_path) :
            #print f_path
            image_list.append(f_path)
            index = index+1
        else:
            #print f_path
            file_found=False
        if index >= end_image_index:
            break
    print 'Found ',len(image_list),' files.'
    return image_list

def find_images(xml_path):
    print 'Finding images in: ',xml_path
    import os.path
    index = xml_path.rfind('/')
    folder = xml_path[0:index+1]
    image_list = []

    file_found = True
    index = start_image_index
    while file_found:
        f_index='';
        f_index=str(index);
        f_index = f_index.zfill(4);
        f_path = folder+'rgb_'+f_index+'.jpg'
        if os.path.isfile(f_path) :
            #print f_path
            image_list.append(f_path)
            index = index+1
        else:
            #print f_path
            file_found=False
        if index >= end_image_index:
            break
    print 'Found ',len(image_list),' files.'
    return image_list

def create_label_file(index, image_name):
    f_index = image_name.rfind('.')
    root = image_name[0:f_index]
    label_file = root+'_label_'+str(index)+'.jpg'
    return label_file

def create_label_file_rgb(index, image_name):
    f_index = image_name.rfind('.')
    root = image_name[0:f_index]
    label_file = root+'_object_'+str(index)+'.jpg'
    return label_file

def find_next_label_number(image_name):
    import os.path

    file_found = True
    index = 0
    while file_found:
        label_file = create_label_file(index, image_name)
        if os.path.isfile(label_file) :
            index=index+1
        else:
            file_found = False

    return index


def onmouse(event,x,y,flags,param):
    global img,img2,drawing,value,mask,rectangle,rect,rect_or_mask,ix,iy,rect_over, depth

    # Draw Rectangle
    if event == cv2.EVENT_RBUTTONDOWN:
        rectangle = True
        ix,iy = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if rectangle == True:
            img = img2.copy()
            cv2.rectangle(img,(ix,iy),(x,y),BLUE,2)
            rect = (min(ix,x),min(iy,y),abs(ix-x),abs(iy-y))
            rect_or_mask = 0

    elif event == cv2.EVENT_RBUTTONUP:
        rectangle = False
        rect_over = True
        cv2.rectangle(img,(ix,iy),(x,y),BLUE,2)
        rect = (min(ix,x),min(iy,y),abs(ix-x),abs(iy-y))
        rect_or_mask = 0
        print " Now press the key 'n' a few times until no further change \n"

    # draw touchup curves

    if event == cv2.EVENT_LBUTTONDOWN:
        if rect_over == False:
            print "first draw rectangle \n"
        else:
            drawing = True
            cv2.circle(img,(x,y),thickness,value['color'],-1)
            cv2.circle(mask,(x,y),thickness,value['val'],-1)
            cv2.circle(depth,(x,y),thickness,value['val'],-1)

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            cv2.circle(img,(x,y),thickness,value['color'],-1)
            cv2.circle(mask,(x,y),thickness,value['val'],-1)
            cv2.circle(depth,(x,y),thickness,value['val'],-1)

    elif event == cv2.EVENT_LBUTTONUP:
        if drawing == True:
            drawing = False
            cv2.circle(img,(x,y),thickness,value['color'],-1)
            cv2.circle(mask,(x,y),thickness,value['val'],-1)
            cv2.circle(depth,(x,y),thickness,value['val'],-1)

# print documentation
#print __doc__

# Loading images
if len(sys.argv) == 4:
    filename = sys.argv[1] # for drawing purposes    
    start_image_index = int(sys.argv[2])
    end_image_index = int(sys.argv[3])
else:
    #print "No input image given, so loading default image, ../data/lena.jpg \n"
    #print "Correct Usage: python grabcut.py <filename> \n"
    filename = '../data/lena.jpg'

sweeps = find_sweeps(sys.argv[1])
images = find_images(sweeps[start_sweep])
#images = find_images('/home/rares/Data/Test_registered_dana//20140922/patrol_run_116/room_2/room.xml')
#depths = find_images_depth('/home/rares/Data/Test_registered_dana//20140922/patrol_run_116/room_2/room.xml')
depths = find_images_depth(sweeps[start_sweep])

img = cv2.imread(images[image_counter])
depth = cv2.imread(depths[image_counter])
img2 = img.copy()                               # a copy of original image
mask = np.zeros(img.shape[:2],dtype = np.uint8) # mask initialized to PR_BG
output = np.zeros(img.shape,np.uint8)           # output image to be shown

# input and output windows
cv2.namedWindow('output')
cv2.namedWindow('input')
cv2.namedWindow('mask')
cv2.namedWindow('depth')
cv2.setMouseCallback('input',onmouse)
cv2.setMouseCallback('depth',onmouse)
cv2.moveWindow('input',img.shape[1]+10,90)

#print " Instructions: \n"
#print " Draw a rectangle around the object using right mouse button \n"

while(1):

    cv2.imshow('output',output)
    cv2.imshow('input',img)
    cv2.imshow('depth',depth)
    k = 0xFF & cv2.waitKey(1)

    # key bindings
    if k == 27:         # esc to exit
        break
    elif k == ord('0'): # BG drawing
        print " mark background regions with left mouse button \n"
        value = DRAW_BG
    elif k == ord('1'): # FG drawing
        print " mark foreground regions with left mouse button \n"
        value = DRAW_FG
    elif k == ord('2'): # PR_BG drawing
        value = DRAW_PR_BG
    elif k == ord('f'): # PR_BG drawing
        label_number = find_next_label_number(images[image_counter])
        label_file = create_label_file(label_number, images[image_counter])
        print 'Next label file is ', label_file
    elif k == ord('.'): # PR_BG drawing
         image_counter=image_counter+1
         if image_counter >= end_image_index:
             # handle by moving to the next sweep
             image_counter = start_image_index;
             current_sweep=current_sweep+1;
             if (current_sweep >= len(sweeps)):
                 current_sweep = len(sweeps)-1
             print 'New sweep number ',current_sweep
             images = find_images(sweeps[current_sweep])
             depths = find_images_depth(sweeps[current_sweep])
	     while len(images) == 0:
		current_sweep = current_sweep+1
             	print 'New sweep number ',current_sweep
             	images = find_images(sweeps[current_sweep])
             	depths = find_images_depth(sweeps[current_sweep])

         img = cv2.imread(images[image_counter])
         depth = cv2.imread(depths[image_counter])
         img2 = img.copy()                               # a copy of original image
         mask = np.zeros(img.shape[:2],dtype = np.uint8) # mask initialized to PR_BG
         output = np.zeros(img.shape,np.uint8)
         label_number=0
         label_number = find_next_label_number(images[image_counter])
         label_file = create_label_file(label_number, images[image_counter])
         print 'Next label file is ', label_file

    elif k == ord(','): # PR_BG drawing
         image_counter=image_counter-1
         if image_counter < start_image_index:
             # handle by moving to previous sweep
             image_counter = start_image_index
             current_sweep=current_sweep-1;
             if (current_sweep < 0):
                 current_sweep = 0
             print 'New sweep number ',current_sweep
             images = find_images(sweeps[current_sweep])
             depths = find_images_depth(sweeps[current_sweep])
	     while len(images) == 0:
		current_sweep = current_sweep-1
             	print 'New sweep number ',current_sweep
             	images = find_images(sweeps[current_sweep])
             	depths = find_images_depth(sweeps[current_sweep])
         img = cv2.imread(images[image_counter])
         depth = cv2.imread(depths[image_counter])
         img2 = img.copy()                               # a copy of original image
         mask = np.zeros(img.shape[:2],dtype = np.uint8) # mask initialized to PR_BG
         output = np.zeros(img.shape,np.uint8)
         label_number=0
         label_number = find_next_label_number(images[image_counter])
         label_file = create_label_file(label_number, images[image_counter])
         print 'Next label file is ', label_file

         #print 'incrementing counter'
    elif k == ord('3'): # PR_FG drawing
        value = DRAW_PR_FG
    elif k == ord('s'): # save image
        bar = np.zeros((img.shape[0],5,3),np.uint8)
        res = np.hstack((img2,bar,img,bar,output))
        #cv2.imwrite('grabcut_output.png',res)
        label_file = create_label_file(label_number, images[image_counter])
        cv2.imwrite(label_file,mask2)
        rgb_file = create_label_file_rgb(label_number, images[image_counter])
        cv2.imwrite(rgb_file,output)
        label_number = label_number + 1
        print " Result saved as image ",label_file
    elif k == ord('r'): # reset everything
        print "resetting \n"
        rect = (0,0,1,1)
        drawing = False
        rectangle = False
        rect_or_mask = 100
        rect_over = False
        value = DRAW_FG
        img = img2.copy()
        mask = np.zeros(img.shape[:2],dtype = np.uint8) # mask initialized to PR_BG
        output = np.zeros(img.shape,np.uint8)           # output image to be shown
    elif k == ord('n'): # segment the image
        print """ For finer touchups, mark foreground and background after pressing keys 0-3
        and again press 'n' \n"""
        if (rect_or_mask == 0):         # grabcut with rect
            bgdmodel = np.zeros((1,65),np.float64)
            fgdmodel = np.zeros((1,65),np.float64)
            cv2.grabCut(img2,mask,rect,bgdmodel,fgdmodel,1,cv2.GC_INIT_WITH_RECT)
            rect_or_mask = 1
        elif rect_or_mask == 1:         # grabcut with mask
            bgdmodel = np.zeros((1,65),np.float64)
            fgdmodel = np.zeros((1,65),np.float64)
            cv2.grabCut(img2,mask,rect,bgdmodel,fgdmodel,1,cv2.GC_INIT_WITH_MASK)


    mask2 = np.where((mask==1) + (mask==3),255,0).astype('uint8')
    cv2.imshow('mask',mask2)
    output = cv2.bitwise_and(img2,img2,mask=mask2)

cv2.destroyAllWindows()
