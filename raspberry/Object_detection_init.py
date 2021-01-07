######## Picamera Object Detection Using Tensorflow Classifier #########
#
# Author: Ane
# Date: 11/03/20
# Description: 
# This program uses a TensorFlow classifier to perform object detection.
# It loads the classifier uses it to perform object detection with the Picamera.
# If the visualization script is used, then boxes around the objects and the scpre pf the classification is shown.

## Some of the code is copied from Google's example at
## https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

## and some is copied from Dat Tran's example at
## https://github.com/datitran/object_detector_app/blob/master/object_detection_app.py



# Import packages
import time
import serial
import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import argparse
import sys

# Set up camera constants
#IM_WIDTH = 1280
#IM_HEIGHT = 720
IM_WIDTH = 640    #Use smaller resolution for
IM_HEIGHT = 480   #slightly faster framerate
N = 5

Init = 0

# Select camera type, by default the picamera is used
camera_type = 'picamera'

# (if user enters --usbcam when calling this script,a USB webcam will be used)
parser = argparse.ArgumentParser()
parser.add_argument('--usbcam', help='Use a USB webcam instead of picamera',
                    action='store_true')
args = parser.parse_args()
if args.usbcam:
    camera_type = 'usb'

# This is needed since the working directory is the object_detection folder.
sys.path.append('..')

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util
from user import connection_user2 as new_user_fun
from functions import functions_main 
#from socket_connection import socket_echo_server as server

# Name of the directory containing the object detection module we're using (NN folder)
MODEL_NAME = 'ssd_mobilenet_v1_ppn_shared_box_predictor_300x300_coco14_sync_2018_07_03'

# Grab path to current working directory
# The  NN folder is in the same place of this script. If the directory of the folder is changed, this one must be changes also
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,'data','mscoco_label_map.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 90

## Load the label map.
# Label maps map indices to category names, so that when the convolution
# network predicts `5`, we know that this corresponds to `airplane`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
# The name of the category, person, airplane...
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
# number of the category: person=1, airplane=5....
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.compat.v1.Session(graph=detection_graph)


# Define input and output tensors (i.e. data) for the object detection classifier

# INPUT
# Input tensor is the image, the frame of the camera is the iput
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# OUTPUT
# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
# Each score represents level of confidence for each of the objects.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
# The score is shown on the result image, together with the class label.
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# Initialize frame rate calculation
frame_rate_calc = 1 # it means it takes one frame per second
freq = cv2.getTickFrequency()
font = cv2.FONT_HERSHEY_SIMPLEX

# Initialize camera and perform object detection.
# The camera has to be set up and used differently depending on if it's a
# Picamera or USB webcam.

# I know this is ugly, but I basically copy+pasted the code for the object
# detection loop twice, and made one work for Picamera and the other work
# for USB.

### Picamera ###
if camera_type == 'picamera':
    # Initialize Picamera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT)) # to obtain the 3D numpy array 
    rawCapture.truncate(0) # to re-use tge rawCapture array
    camera.capture(rawCapture, format="bgr",use_video_port=True)
    
    #Analyze if we are tracking auser or not
    tracking_a_user = False
    actual_action = " "
    action_sound = "none"
    
    # number of frames to analyze human
    loop = 4
    cont_int = 0
    action_count = 1
    # create the connection with the computer
    #s = server.setupServer()
    # set the connection with the computer
    #conn = server.setupConnection(s)

    # here it is cretaed the continious loop,
    while True:
            angle_tot = 0 
            angle_mean = 0
            count = 0
            
            new_user_fun.new_user_function()
            if (new_user_fun.state_system == "busy"):
                #functions_main.reproduce_action_sound(action_sound)
                print("system is busy")
            #server.dataTransfer(conn,s)
            #actual_action = functions_main.send_action_arduino(actual_action, server.reply, new_user_fun.ser, tracking_a_user)
            else:
                if (Init != 2):
                    if (new_user_fun.old_user != "none"):
                        t1 = cv2.getTickCount()
                        
                        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
                        # i.e. a single-column array, where each item in the column has the pixel RGB value
                        frame = np.copy(rawCapture.array)
                        frame.setflags(write=1)
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame_expanded = np.expand_dims(frame_rgb, axis=0)

                        # Perform the actual detection by running the model with the image as input
                        (boxes, scores, classes, num) = sess.run(
                            [detection_boxes, detection_scores, detection_classes, num_detections],
                            feed_dict={image_tensor: frame_expanded})
                        
                        # Draw the results of the detection (aka 'visulaize the results')
                        #obtain the angle
                        angle = vis_util.user_orientation(
                            frame,
                            np.squeeze(boxes),
                            np.squeeze(classes).astype(np.int32),
                            np.squeeze(scores),
                            category_index,
                            use_normalized_coordinates=True,
                            line_thickness=8,
                            min_score_thresh=0.42)
                        if (angle != 0):
                            print("angle: " + str(angle))
                            count = 4;
                            tracking_a_user  = functions_main.human_detection(angle, new_user_fun.old_user,count)
                            if(tracking_a_user==True):
                                Init =1
                                if(vis_util.there_is_an_object==True):
                                    cont_int = 0
                                    if(action_count >=3):
                                        action_count = 1
                                    else:
                                        action_count =  action_count+1
                                    functions_main.send_initial_action_arduino(("init" +str(action_count)), new_user_fun.ser,"out")
                                    action_sound = "out"
                                    #functions_main.reproduce_action_sound(action_sound)
                                    print("OBJEEEEEEEECT")
                                    print("go out child")
                                else:
                                    if(action_count >=2):
                                        action_count = 1
                                    functions_main.send_initial_action_arduino(("init" +str(action_count)), new_user_fun.ser,"excited")
                                    cont_int = cont_int+1
                                    action_sound = "excited"
                                    #functions_main.reproduce_action_sound(action_sound)
                                    print("No object")
                                if (cont_int>2):
                                    Init = 2
                                    print("human free")
                            else:
                                cont_int = 0
                                if (angle>=0):
                                    functions_main.send_initial_action_arduino("rotateRight", new_user_fun.ser,"none")
                                else:
                                    functions_main.send_initial_action_arduino("rotateLeft", new_user_fun.ser,"none")
                                print("move laterally to the human")
                            
                        else:
                            functions_main.send_initial_action_arduino("rotate", new_user_fun.ser, "found")
                            action_sound = "found"
                            #functions_main.reproduce_action_sound(action_sound)
                            print("rotate find the child")
                            

                        #cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

                        # All the results have been drawn on the frame, so it's time to display it.
                        #cv2.imshow('Object detector', frame)

                        t2 = cv2.getTickCount()
                        time1 = (t2-t1)/freq
                        frame_rate_calc = 1/time1

                        # Press 'q' to quit
                        if cv2.waitKey(1) == ord('q'):
                            break

                        rawCapture.truncate(0)
                        frame1 = camera.capture(rawCapture, format="bgr",use_video_port=True)
                    else:
                        #print("entered none")
                        cont_int = 0
                        #print(new_user_fun.is_new_user)  
                        t1 = cv2.getTickCount()
                        
                        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
                        # i.e. a single-column array, where each item in the column has the pixel RGB value
                        frame = np.copy(rawCapture.array)
                        frame.setflags(write=1)
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame_expanded = np.expand_dims(frame_rgb, axis=0)

                        # Perform the actual detection by running the model with the image as input
                        (boxes, scores, classes, num) = sess.run(
                            [detection_boxes, detection_scores, detection_classes, num_detections],
                            feed_dict={image_tensor: frame_expanded})
                        
                        # Draw the results of the detection (aka 'visulaize the results')
                        #obtain the angle
                        angle = vis_util.user_orientation(
                            frame,
                            np.squeeze(boxes),
                            np.squeeze(classes).astype(np.int32),
                            np.squeeze(scores),
                            category_index,
                            use_normalized_coordinates=True,
                            line_thickness=8,
                            min_score_thresh=0.42)
                        if (angle != 0):
                            print("angle: " + str(angle))
                            Init = 1
                            #tracking_a_user  = functions_main.human_detection(angle, new_user_fun.old_user,count)
                            #if(tracking_a_user==True):
                            if (abs(angle) < 15):
                                functions_main.send_initial_action_arduino("move", new_user_fun.ser,"move")
                                action_sound = "move"
                                #functions_main.reproduce_action_sound(action_sound)
                                print("move")
                            else:
                                if (angle>=0):
                                    functions_main.send_initial_action_arduino("rotateRight", new_user_fun.ser,"none")
                                else:
                                    functions_main.send_initial_action_arduino("rotateLeft", new_user_fun.ser,"none")
                            print("move to the human's position")     
                        else:
                            functions_main.send_initial_action_arduino("rotate", new_user_fun.ser,"found")
                            action_sound = "found"
                            #functions_main.reproduce_action_sound(action_sound)
                            print("rotar find the child")
                            

                        #cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

                        # All the results have been drawn on the frame, so it's time to display it.
                        #cv2.imshow('Object detector', frame)

                        t2 = cv2.getTickCount()
                        time1 = (t2-t1)/freq
                        frame_rate_calc = 1/time1

                        # Press 'q' to quit
                        if cv2.waitKey(1) == ord('q'):
                            break

                        rawCapture.truncate(0)
                        frame1 = camera.capture(rawCapture, format="bgr",use_video_port=True)
            
                        
        #            print("Arduino :"+ line)
        #            print('\n')
                    #time.sleep(1)
                elif(new_user_fun.old_user == "none"):
                    tracking_a_user = False
                    actual_action = " "
                
    server.conn.close()
