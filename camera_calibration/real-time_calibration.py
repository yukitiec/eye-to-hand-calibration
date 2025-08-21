import cv2 as cv
import glob
import numpy as np
import sys
from scipy import linalg
import yaml
from ximea import xiapi
import os
from sys import platform
import argparse
import time

#This will contain the calibration settings from the calibration_settings.yaml file
calibration_settings = {}
bool_downsample=True

#Given Projection matrices P1 and P2, and pixel coordinates point1 and point2, return triangulated 3D point.
def DLT(P1, P2, point1, point2):

    A = [point1[1]*P1[2,:] - P1[1,:],
         P1[0,:] - point1[0]*P1[2,:],
         point2[1]*P2[2,:] - P2[1,:],
         P2[0,:] - point2[0]*P2[2,:]
        ]
    A = np.array(A).reshape((4,4))

    B = A.transpose() @ A
    U, s, Vh = linalg.svd(B, full_matrices = False)

    #print('Triangulated point: ')
    #print(Vh[3,0:3]/Vh[3,3])
    return Vh[3,0:3]/Vh[3,3]


#Open and load the calibration_settings.yaml file
def parse_calibration_settings_file(filename):
    
    global calibration_settings

    if not os.path.exists(filename):
        print('File does not exist:', filename)
        quit()
    
    print('Using for calibration settings: ', filename)

    with open(filename) as f:
        calibration_settings = yaml.safe_load(f)

    #rudimentray check to make sure correct file was loaded
    if 'camera0' not in calibration_settings.keys():
        print('camera0 key was not found in the settings file. Check if correct calibration_settings.yaml file was passed')
        quit()


#Open camera stream and save frames
def save_frames_single_camera(camera_name, devnum):

    #create frames directory
    if not os.path.exists('frames'):
        os.mkdir('frames')

    #get settings
    camera_device_id = calibration_settings[camera_name]
    CAMERAS_ON_SAME_CONTROLLER = 2

    #create instance for cameras
    cam1 = xiapi.Camera(dev_id=devnum)

    #start communication
    print('Opening cameras...')
    cam1.open_device_by_SN(camera_device_id)

    #set interface data rate
    interface_data_rate=cam1.get_limit_bandwidth()
    camera_data_rate = int(interface_data_rate / CAMERAS_ON_SAME_CONTROLLER)

    #set data rate
    cam1.set_limit_bandwidth(camera_data_rate)

    #print device serial numbers
    print('Camera 1 serial number: ' + str(cam1.get_device_sn()))


    # create instance for first connected camera
    #cam = xiapi.Camera()

    # start communication
    #print('Opening first camera...')
    #cam.open_device()
    # settings
    #downsampling
    if bool_downsample:
        cam1.set_downsampling('XI_DWN_2x2')
        cam1.set_downsampling_type('XI_SKIPPING')
        cam1.set_width(512)
        cam1.set_height(512)
        cam1.set_offsetX(64)
        #cam1.set_offsetY(192)
    else:        
        print('The maximal width of this camera is %i.' %cam1.get_width_maximum())
        print('The minimal width of this camera is %i.' %cam1.get_width_minimum())
        print('The increment of the width of this camera is %i.' %cam1.get_width_increment())
        print('The maximal height of this camera is %i.' %cam1.get_height_maximum())
        print('The minimal height of this camera is %i.' %cam1.get_height_minimum())
        print('The increment of the height of this camera is %i.' %cam1.get_height_increment())
        cam1.set_downsampling('XI_DWN_1x1')
        cam1.set_downsampling_type('XI_SKIPPING')
        cam1.set_width(800)
        cam1.set_height(800)
        cam1.set_offsetX(240)
        cam1.set_offsetY(112)
    #cam.set_limit_bandwidth_mode()

    cam1.set_exposure(2300)
    cam1.set_gain(20.0)
    cam1.set_imgdataformat('XI_MONO8')

    # create instance of Image to store image data and metadata
    img1 = xiapi.Image()

    # start data acquisition
    print('Starting data acquisition...')
    cam1.start_acquisition()

    number_to_save = calibration_settings['mono_calibration_frames']
    view_resize = calibration_settings['view_resize']
    cooldown_time = calibration_settings['cooldown']

    #open video stream and change resolution.
    #Note: if unsupported resolution is used, this does NOT raise an error.
    #cap = cv.VideoCapture(camera_device_id)
    # cap.set(3, width)
    # cap.set(4, height)

    cooldown = cooldown_time
    start = False
    saved_count = 0

    while True:
        cam1.get_image(img1)

        # create numpy array with data from camera. Dimensions of the array are
        # determined by imgdataformat
        data1 = img1.get_image_data_numpy()

        # ret, frame = cap.read()
        # if ret == False:
        #     #if no video data is received, can't calibrate the camera, so exit.
        #     print("No video data received from camera. Exiting...")
        #     quit()

        frame_small = cv.resize(data1, None, fx = 1/view_resize, fy=1/view_resize)

        if not start:
            cv.putText(frame_small, "Press SPACEBAR to start collection frames", (50,50), cv.FONT_HERSHEY_COMPLEX, 0.3, (0,0,255), 1)
        
        if start:
            cooldown -= 1
            cv.putText(frame_small, "Cooldown: " + str(cooldown), (50,50), cv.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 1)
            cv.putText(frame_small, "Num frames: " + str(saved_count), (50,100), cv.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 1)
            
            #save the frame when cooldown reaches 0.
            if cooldown <= 0:
                savename = os.path.join('frames', camera_name + '_' + str(saved_count) + '.png')
                #print(data1.shape)
                cv.imwrite(savename, data1)
                saved_count += 1
                cooldown = cooldown_time

        cv.imshow('frame_small', frame_small)
        k = cv.waitKey(1)
        
        if k == 27:
            #if ESC is pressed at any time, the program will exit.
            quit()

        if k == 32:
            #Press spacebar to start data collection
            start = True

        #break out of the loop when enough number of frames have been saved
        if saved_count == number_to_save: break

    cv.destroyAllWindows()


#Calibrate single camera to obtain camera intrinsic parameters from saved frames.
def calibrate_camera_for_intrinsic_parameters(images_prefix):
    
    #NOTE: images_prefix contains camera name: "frames/camera0*".
    images_names = glob.glob(images_prefix)

    #read all frames
    images = [cv.imread(imname, 1) for imname in images_names]

    #criteria used by checkerboard pattern detector.
    #Change this if the code can't find the checkerboard. 
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

    rows = calibration_settings['checkerboard_rows']
    columns = calibration_settings['checkerboard_columns']
    world_scaling = calibration_settings['checkerboard_box_size_scale'] #this will change to user defined length scale

    #coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns,3), np.float32)
    objp[:,:2] = np.mgrid[0:rows,0:columns].T.reshape(-1,2)
    objp = world_scaling* objp

    #frame dimensions. Frames should be the same size.
    width = images[0].shape[1]
    height = images[0].shape[0]

    #Pixel coordinates of checkerboards
    imgpoints = [] # 2d points in image plane.

    #coordinates of the checkerboard in checkerboard world space.
    objpoints = [] # 3d point in real world space


    for i, frame in enumerate(images):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        #find the checkerboard
        ret, corners = cv.findChessboardCorners(gray, (rows, columns), None)

        if ret == True:

            #Convolution size used to improve corner detection. Don't make this too large.
            conv_size = (5, 5)

            #opencv can attempt to improve the checkerboard coordinates
            corners = cv.cornerSubPix(gray, corners, conv_size, (-1, -1), criteria)
            cv.drawChessboardCorners(frame, (rows,columns), corners, ret)
            cv.putText(frame, 'If detected points are poor, press "s" to skip this sample', (25, 25), cv.FONT_HERSHEY_COMPLEX, 0.3, (0,0,255), 1)

            cv.imshow('img', frame)
            k = cv.waitKey(0)

            if k & 0xFF == ord('s'):
                print('skipping')
                continue

            objpoints.append(objp)
            imgpoints.append(corners)


    cv.destroyAllWindows()
    ret, cmtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (width, height), None, None)
    print('rmse:', ret)
    print('camera matrix:\n', cmtx)
    print('distortion coeffs:', dist)

    return cmtx, dist

#save camera intrinsic parameters to file
def save_camera_intrinsics(camera_matrix, distortion_coefs, camera_name):

    #create folder if it does not exist
    if not os.path.exists('camera_parameters'):
        os.mkdir('camera_parameters')

    out_filename = os.path.join('camera_parameters', camera_name + '_intrinsics.dat')
    outf = open(out_filename, 'w')

    outf.write('intrinsic:\n')
    for l in camera_matrix:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')

    outf.write('distortion:\n')
    for en in distortion_coefs[0]:
        outf.write(str(en) + ' ')
    outf.write('\n')


#open both cameras and take calibration frames
def save_frames_two_cams(camera0_name, devnum1, camera1_name, devnum2):

    #create frames directory
    if not os.path.exists('frames_pair'):
        os.mkdir('frames_pair')

    #settings for taking data
    view_resize = calibration_settings['view_resize']
    cooldown_time = calibration_settings['cooldown']    
    number_to_save = calibration_settings['stereo_calibration_frames']

    #open the video streams
    # cap0 = cv.VideoCapture(calibration_settings[camera0_name])
    # cap1 = cv.VideoCapture(calibration_settings[camera1_name])

    #set camera resolutions
    # width = calibration_settings['frame_width']
    # height = calibration_settings['frame_height']
    # cap0.set(3, width)
    # cap0.set(4, height)
    # cap1.set(3, width)
    # cap1.set(4, height)

    CAMERAS_ON_SAME_CONTROLLER = 2

    #create instance for cameras
    cam1 = xiapi.Camera(dev_id=devnum1)
    cam2 = xiapi.Camera(dev_id=devnum2)
    #start communication
    print('Opening cameras...')
    cam1.open_device_by_SN(calibration_settings[camera0_name])
    cam2.open_device_by_SN(calibration_settings[camera1_name])

    # Get data rate from camera
    interface_data_rate = cam1.get_limit_bandwidth()

    # Compute shared bandwidth
    camera_data_rate = int(interface_data_rate / CAMERAS_ON_SAME_CONTROLLER)

    # Validate range
    #min_bw = cam1.get_param('limit_bandwidth_min')
    #max_bw = cam1.get_param('limit_bandwidth_max')
    #camera_data_rate = max(min(camera_data_rate, max_bw), min_bw)
    print(f"Setting camera bandwidth to {camera_data_rate}")

    # Set bandwidth
    cam1.set_limit_bandwidth(camera_data_rate)
    cam2.set_limit_bandwidth(camera_data_rate)

    #print device serial numbers
    print('Camera 1 serial number: ' + str(cam1.get_device_sn()))
    print('Camera 2 serial number: ' + str(cam2.get_device_sn()))

    # create instance for first connected camera
    #cam = xiapi.Camera()

    # start communication
    #print('Opening first camera...')
    #cam.open_device()

    # settings
    #downsampling
    if bool_downsample:
        cam1.set_downsampling('XI_DWN_2x2')
        cam1.set_downsampling_type('XI_SKIPPING')
        cam1.set_width(512)
        cam1.set_height(512)
        cam1.set_offsetX(64)
        #cam1.set_offsetY(192)
    else:        
        cam1.set_downsampling('XI_DWN_1x1')
        cam1.set_downsampling_type('XI_SKIPPING')
        cam1.set_width(800)
        cam1.set_height(800)
        cam1.set_offsetX(240)
        cam1.set_offsetY(112)
    #cam.set_limit_bandwidth_mode()

    cam1.set_exposure(2300)
    cam1.set_gain(20.0)
    cam1.set_imgdataformat('XI_MONO8')

    #cam2
    if bool_downsample:
        cam2.set_downsampling('XI_DWN_2x2')
        cam2.set_downsampling_type('XI_SKIPPING')
        cam2.set_width(512)
        cam2.set_height(512)
        #for downsampling to 512*512
        cam2.set_offsetX(64)
        #cam2.set_offsetY(192)
    else:    
        cam2.set_downsampling('XI_DWN_1x1')
        cam2.set_downsampling_type('XI_SKIPPING')
        cam2.set_width(800)
        cam2.set_height(800)
        #for 640*640
        cam2.set_offsetX(240)
        cam2.set_offsetY(112)
    #cam.set_limit_bandwidth_mode()

    cam2.set_exposure(2300)
    cam2.set_gain(20.0)
    cam2.set_imgdataformat('XI_MONO8')


    # create instance of Image to store image data and metadata
    img1 = xiapi.Image()
    img2 = xiapi.Image()

    # start data acquisition
    print('Starting data acquisition...')

    number_to_save = calibration_settings['mono_calibration_frames']
    view_resize = calibration_settings['view_resize']
    cooldown_time = calibration_settings['cooldown']

    #open video stream and change resolution.
    #Note: if unsupported resolution is used, this does NOT raise an error.
    #cap = cv.VideoCapture(camera_device_id)
    # cap.set(3, width)
    # cap.set(4, height)
    

    cooldown = cooldown_time
    start = False
    saved_count = 0
    while True:
        cam1.get_image(img1)
        cam2.get_image(img2)

            # create numpy array with data from camera. Dimensions of the array are
            # determined by imgdataformat
        data1 = img1.get_image_data_numpy()
        data2 = img2.get_image_data_numpy()
        #ret0, frame0 = cap0.read()
        #ret1, frame1 = cap1.read()

        # if not ret0 or not ret1:
        #     print('Cameras not returning video data. Exiting...')
        #     quit()

        frame0_small = cv.resize(data1, None, fx=1./view_resize, fy=1./view_resize)
        frame1_small = cv.resize(data2, None, fx=1./view_resize, fy=1./view_resize)

        if not start:
            cv.putText(frame0_small, "Make sure both cameras can see the calibration pattern well", (50,50), cv.FONT_HERSHEY_COMPLEX, 0.3, (0,0,255), 1)
            cv.putText(frame0_small, "Press SPACEBAR to start collection frames", (50,100), cv.FONT_HERSHEY_COMPLEX, 0.3, (0,0,255), 1)
        
        if start:
            cooldown -= 1
            cv.putText(frame0_small, "Cooldown: " + str(cooldown), (50,50), cv.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 1)
            cv.putText(frame0_small, "Num frames: " + str(saved_count), (50,100), cv.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 1)
            
            cv.putText(frame1_small, "Cooldown: " + str(cooldown), (50,50), cv.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 1)
            cv.putText(frame1_small, "Num frames: " + str(saved_count), (50,100), cv.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 1)

            #save the frame when cooldown reaches 0.
            if cooldown <= 0:
                savename = os.path.join('frames_pair', camera0_name + '_' + str(saved_count) + '.png')
                cv.imwrite(savename, data1)

                savename = os.path.join('frames_pair', camera1_name + '_' + str(saved_count) + '.png')
                cv.imwrite(savename, data2)

                saved_count += 1
                cooldown = cooldown_time

        cv.imshow('frame0_small', frame0_small)
        cv.imshow('frame1_small', frame1_small)
        k = cv.waitKey(1)
        
        if k == 27:
            #if ESC is pressed at any time, the program will exit.
            quit()

        if k == 32:
            #Press spacebar to start data collection
            start = True

        #break out of the loop when enough number of frames have been saved
        if saved_count == number_to_save: break

    cv.destroyAllWindows()


#open paired calibration frames and stereo calibrate for cam0 to cam1 coorindate transformations
def stereo_calibrate(mtx0, dist0, mtx1, dist1, frames_prefix_c0, frames_prefix_c1):
    #read the synched frames
    c0_images_names = sorted(glob.glob(frames_prefix_c0))
    c1_images_names = sorted(glob.glob(frames_prefix_c1))

    #open images
    c0_images = [cv.imread(imname, 1) for imname in c0_images_names]
    c1_images = [cv.imread(imname, 1) for imname in c1_images_names]

    #change this if stereo calibration not good.
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

    #calibration pattern settings
    rows = calibration_settings['checkerboard_rows']
    columns = calibration_settings['checkerboard_columns']
    world_scaling = calibration_settings['checkerboard_box_size_scale']

    #coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns,3), np.float32)
    objp[:,:2] = np.mgrid[0:rows,0:columns].T.reshape(-1,2)
    objp = world_scaling* objp

    #frame dimensions. Frames should be the same size.
    width = c0_images[0].shape[1]
    height = c0_images[0].shape[0]

    #Pixel coordinates of checkerboards
    imgpoints_left = [] # 2d points in image plane.
    imgpoints_right = []

    #coordinates of the checkerboard in checkerboard world space.
    objpoints = [] # 3d point in real world space

    for frame0, frame1 in zip(c0_images, c1_images):
        gray1 = cv.cvtColor(frame0, cv.COLOR_BGR2GRAY)
        gray2 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
        c_ret1, corners1 = cv.findChessboardCorners(gray1, (rows, columns), None)
        c_ret2, corners2 = cv.findChessboardCorners(gray2, (rows, columns), None)

        if c_ret1 == True and c_ret2 == True:

            corners1 = cv.cornerSubPix(gray1, corners1, (5, 5), (-1, -1), criteria)
            corners2 = cv.cornerSubPix(gray2, corners2, (5, 5), (-1, -1), criteria)

            p0_c1 = corners1[0,0].astype(np.int32)
            p0_c2 = corners2[0,0].astype(np.int32)

            cv.putText(frame0, 'O', (p0_c1[0], p0_c1[1]), cv.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 1)
            cv.drawChessboardCorners(frame0, (rows,columns), corners1, c_ret1)
            cv.imshow('img', frame0)

            cv.putText(frame1, 'O', (p0_c2[0], p0_c2[1]), cv.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 1)
            cv.drawChessboardCorners(frame1, (rows,columns), corners2, c_ret2)
            cv.imshow('img2', frame1)
            k = cv.waitKey(0)

            if k & 0xFF == ord('s'):
                print('skipping')
                continue

            objpoints.append(objp)
            imgpoints_left.append(corners1)
            imgpoints_right.append(corners2)

    stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
    ret, CM1, dist0, CM2, dist1, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx0, dist0,mtx1, dist1, (width, height), criteria = criteria, flags = stereocalibration_flags)

    print('rmse: ', ret)
    cv.destroyAllWindows()
    return R, T

#Converts Rotation matrix R and Translation vector T into a homogeneous representation matrix
def _make_homogeneous_rep_matrix(R, t):
    P = np.zeros((4,4))
    P[:3,:3] = R
    P[:3, 3] = t.reshape(3)
    P[3,3] = 1
 
    return P
# Turn camera calibration data into projection matrix
def get_projection_matrix(cmtx, R, T):
    P = cmtx @ _make_homogeneous_rep_matrix(R, T)[:3,:]
    return P

# After calibrating, we can see shifted coordinate axes in the video feeds directly
def check_calibration(camera0_name, devnum1, camera0_data, camera1_name, devnum2, camera1_data, _zshift = 50.):
    
    cmtx0 = np.array(camera0_data[0])
    dist0 = np.array(camera0_data[1])
    R0 = np.array(camera0_data[2])
    T0 = np.array(camera0_data[3])
    cmtx1 = np.array(camera1_data[0])
    dist1 = np.array(camera1_data[1])
    R1 = np.array(camera1_data[2])
    T1 = np.array(camera1_data[3])

    P0 = get_projection_matrix(cmtx0, R0, T0)
    P1 = get_projection_matrix(cmtx1, R1, T1)

    #define coordinate axes in 3D space. These are just the usual coorindate vectors
    coordinate_points = np.array([[0.,0.,0.],
                                  [1.,0.,0.],
                                  [0.,1.,0.],
                                  [0.,0.,1.]])
    z_shift = np.array([0.,0.,_zshift]).reshape((1, 3))
    #increase the size of the coorindate axes and shift in the z direction
    draw_axes_points = 5 * coordinate_points + z_shift

    #project 3D points to each camera view manually. This can also be done using cv.projectPoints()
    #Note that this uses homogenous coordinate formulation
    pixel_points_camera0 = []
    pixel_points_camera1 = []
    for _p in draw_axes_points:
        X = np.array([_p[0], _p[1], _p[2], 1.])
        
        #project to camera0
        uv = P0 @ X
        uv = np.array([uv[0], uv[1]])/uv[2]
        pixel_points_camera0.append(uv)

        #project to camera1
        uv = P1 @ X
        uv = np.array([uv[0], uv[1]])/uv[2]
        pixel_points_camera1.append(uv)

    #these contain the pixel coorindates in each camera view as: (pxl_x, pxl_y)
    pixel_points_camera0 = np.array(pixel_points_camera0)
    pixel_points_camera1 = np.array(pixel_points_camera1)

    CAMERAS_ON_SAME_CONTROLLER = 2

    #create instance for cameras
    cam1 = xiapi.Camera(dev_id=devnum1)
    cam2 = xiapi.Camera(dev_id=devnum2)
    #start communication
    print('Opening cameras...')
    cam1.open_device_by_SN(calibration_settings[camera0_name])
    cam2.open_device_by_SN(calibration_settings[camera1_name])

    #set interface data rate
    interface_data_rate=cam1.get_limit_bandwidth()
    camera_data_rate = int(interface_data_rate / CAMERAS_ON_SAME_CONTROLLER)

    #set data rate
    #cam1.set_limit_bandwidth(50)#camera_data_rate)
    #cam2.set_limit_bandwidth(50)#camera_data_rate)

    #print device serial numbers
    print('Camera 1 serial number: ' + str(cam1.get_device_sn()))
    print('Camera 2 serial number: ' + str(cam2.get_device_sn()))

    # create instance for first connected camera
    #cam = xiapi.Camera()

    # start communication
    #print('Opening first camera...')
    #cam.open_device()

    # settings
    #downsampling
    if bool_downsample:
        cam1.set_downsampling('XI_DWN_2x2')
        cam1.set_downsampling_type('XI_SKIPPING')
        cam1.set_width(512)
        cam1.set_height(512)
        cam1.set_offsetX(64)
        #cam1.set_offsetY(192)
    else:      
        cam1.set_downsampling('XI_DWN_1x1')  
        cam1.set_downsampling_type('XI_SKIPPING')
        cam1.set_width(800)
        cam1.set_height(800)
        cam1.set_offsetX(240)
        cam1.set_offsetY(112)
    #cam.set_limit_bandwidth_mode()

    cam1.set_exposure(2300)
    cam1.set_gain(20.0)
    cam1.set_imgdataformat('XI_MONO8')

    #cam2 :512*512
    if bool_downsample:
        cam2.set_downsampling('XI_DWN_2x2')
        cam2.set_downsampling_type('XI_SKIPPING')
        cam2.set_width(512)
        cam2.set_height(512)
        #for downsampling to 512*512
        cam2.set_offsetX(64)
        #cam2.set_offsetY(192)
    else:    
        cam2.set_downsampling('XI_DWN_1x1')
        cam2.set_downsampling_type('XI_SKIPPING')
        cam2.set_width(800)
        cam2.set_height(800)
        #for 640*640
        cam2.set_offsetX(240)
        cam2.set_offsetY(112)
    #cam.set_limit_bandwidth_mode()

    cam2.set_exposure(2300)
    cam2.set_gain(20.0)
    cam2.set_imgdataformat('XI_MONO8')


    # create instance of Image to store image data and metadata
    img1 = xiapi.Image()
    img2 = xiapi.Image()

    # start data acquisition
    
    #open the video streams
    # cap0 = cv.VideoCapture(calibration_settings[camera0_name])
    # cap1 = cv.VideoCapture(calibration_settings[camera1_name])

    # #set camera resolutions
    # width = calibration_settings['frame_width']
    # height = calibration_settings['frame_height']
    # cap0.set(3, width)
    # cap0.set(4, height)
    # cap1.set(3, width)
    # cap1.set(4, height)

    count = 0
    bool_template = True
    time.sleep(5)
    while True:
        #print(count)
        #print('Starting data acquisition...')

        #open video stream and change resolution.
        #Note: if unsupported resolution is used, this does NOT raise an error.
        #cap = cv.VideoCapture(camera_device_id)
        # cap.set(3, width)
        # cap.set(4, height)
        cam1.get_image(img1)
        cam2.get_image(img2)

            # create numpy array with data from camera. Dimensions of the array are
            # determined by imgdataformat
        data1 = img1.get_image_data_numpy()
        data2 = img2.get_image_data_numpy()
        # ret0, frame0 = cap0.read()
        # ret1, frame1 = cap1.read()

        # if not ret0 or not ret1:
        #     print('Video stream not returning frame data')
        #     quit()
        #rint("2")
        if bool_template:
            if (count==0):
                if len(data1.shape) == 3 and len(data2.shape)==3:
                    data1 = cv.cvtColor(data1, cv.COLOR_BGR2GRAY)
                    data2 = cv.cvtColor(data2, cv.COLOR_BGR2GRAY)
                # Display the image
                cv.imshow('Select ROI in left',data1)
                # Call selectROI on the image window
                roi1 = cv.selectROI('Select ROI in left', data1, fromCenter=False)
                # Crop the selected ROI from the image
                template1 = data1[int(roi1[1]):int(roi1[1]+roi1[3]), int(roi1[0]):int(roi1[0]+roi1[2])]
                # Display the image
                cv.imshow('Select ROI in right',data2)
                # Call selectROI on the image window
                roi2 = cv.selectROI('Select ROI in right', data2, fromCenter=False)
                # Crop the selected ROI from the image
                template2 = data2[int(roi2[1]):int(roi2[1]+roi2[3]), int(roi2[0]):int(roi2[0]+roi2[2])]
                count+=1
                #cv.destroyAllWindows()
                continue
            if template1.shape[0]==0 or template2.shape[0] == 0:
                count=0
                continue
            #print("3")
            #calculate depth image
            # Convert images to grayscale
            if len(data1.shape) == 3 and len(data2.shape)==3:
                gray1 = cv.cvtColor(data1, cv.COLOR_BGR2GRAY)
                gray2 = cv.cvtColor(data2, cv.COLOR_BGR2GRAY)
            else:
                gray1 = data1
                gray2 = data2
            #print("3")
            # Perform template matching
            result1 = cv.matchTemplate(gray1, template1, cv.TM_CCOEFF_NORMED)
            result2 = cv.matchTemplate(gray2, template2, cv.TM_CCOEFF_NORMED)

            # Get the minimum and maximum values in the result matrix and their corresponding locations
            min_val1, max_val1, min_loc1, max_loc1 = cv.minMaxLoc(result1)
            min_val2, max_val2, min_loc2, max_loc2 = cv.minMaxLoc(result2)
            print("maxval1=",max_val1,", maxval2=",max_val2)
            #print("4")
            if max_val1>0.0 and max_val2>0.0:
                print(max_loc1,max_loc2)
                # Draw a rectangle around the best match
                best_match_loc1 = max_loc1
                best_match_loc2 = max_loc2
                template1_height, template1_width = template1.shape[::-1]  # Get template dimensions
                template2_height, template2_width = template2.shape[::-1]  # Get template dimensions
                #print("2")
                cv.rectangle(gray1, best_match_loc1, (best_match_loc1[0] + template1_width, best_match_loc1[1] + template1_height), (0, 255, 0), 2)
                cv.rectangle(gray2, best_match_loc2, (best_match_loc2[0] + template2_width, best_match_loc2[1] + template2_height), (0, 255, 0), 2)
                #print("3")
                center1 = np.array([best_match_loc1[0] + template1_width/2, best_match_loc1[1] + template1_height/2]) #x,y
                center2 = np.array([best_match_loc2[0] + template2_width, best_match_loc2[1] + template2_height]) #x,ycv.
                center1 = center1.reshape([-1,1,2]).astype("float")
                center2 = center2.reshape([-1,1,2]).astype("float")
                center1 = cv.undistortPoints(center1, cmtx0, dist0)
                center2 = cv.undistortPoints(center2, cmtx1, dist1)
                # Extract the undistorted coordinates
                center1 = center1[:, 0, :][0]
                center2 = center2[:, 0, :][0]
                points_3d_homogeneous = cv.triangulatePoints(P0,P1, center1,center2)
                # Convert homogeneous coordinates to Cartesian coordinates by dividing by the last element
                point3d = points_3d_homogeneous / points_3d_homogeneous[3]
                # Extract the 3D points
                point3d = (point3d[:3].T)[0]
                #print(P0,P1)
                #point3d = DLT(P0,P1,center1,center2) #triangulate
                print(point3d)
                # Define the text and font properties
                text = "z="+str(point3d[2])+"mm"
                print(text)
                font = cv.FONT_HERSHEY_SIMPLEX
                font_scale = 1

                font_color = (255, 255, 255)  # White color in BGR
                thickness = 1
                # Get the size of the text
                (text_width, text_height), baseline = cv.getTextSize(text, font, font_scale, thickness)
                # Calculate the position to put the text (bottom-left corner)
                text_position = (10, gray1.shape[0] - 10)
                # Put the text on the image
                cv.putText(gray1, text, text_position, font, font_scale, font_color, thickness)
                cv.putText(gray2, text, text_position, font, font_scale, font_color, thickness)
        cv.imshow("frame0",gray1)
        cv.imshow("frame1",gray2)
        k = cv.waitKey(1)
        if k == 27: break
        """        
        #follow RGB colors to indicate XYZ axes respectively
        colors = [(0,0,255), (0,255,0), (255,0,0)]
        #draw projections to camera0
        origin = tuple(pixel_points_camera0[0].astype(np.int32))
        for col, _p in zip(colors, pixel_points_camera0[1:]):
            _p = tuple(_p.astype(np.int32))
            cv.line(data1, origin, _p, col, 2)
        
        #draw projections to camera1
        origin = tuple(pixel_points_camera1[0].astype(np.int32))
        for col, _p in zip(colors, pixel_points_camera1[1:]):
            _p = tuple(_p.astype(np.int32))
            cv.line(data2, origin, _p, col, 2)

        cv.imshow('frame0', data1)
        cv.imshow('frame1', data2)
        """

    cv.destroyAllWindows()

def get_world_space_origin(cmtx, dist, img_path):

    frame = cv.imread(img_path, 1)

    #calibration pattern settings
    rows = calibration_settings['checkerboard_rows']
    columns = calibration_settings['checkerboard_columns']
    world_scaling = calibration_settings['checkerboard_box_size_scale']

    #coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns,3), np.float32)
    objp[:,:2] = np.mgrid[0:rows,0:columns].T.reshape(-1,2)
    objp = world_scaling* objp

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (rows, columns), None)

    cv.drawChessboardCorners(frame, (rows,columns), corners, ret)
    cv.putText(frame, "If you don't see detected points, try with a different image", (50,50), cv.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 1)
    cv.imshow('img', frame)
    cv.waitKey(0)

    ret, rvec, tvec = cv.solvePnP(objp, corners, cmtx, dist)
    R, _  = cv.Rodrigues(rvec) #rvec is Rotation matrix in Rodrigues vector form

    return R, tvec

def get_cam1_to_world_transforms(cmtx0, dist0, R_W0, T_W0, 
                                 cmtx1, dist1, R_01, T_01,
                                 image_path0,
                                 image_path1):

    frame0 = cv.imread(image_path0, 1)
    frame1 = cv.imread(image_path1, 1)

    unitv_points = 5 * np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))
    #axes colors are RGB format to indicate XYZ axes.
    colors = [(0,0,255), (0,255,0), (255,0,0)]

    #project origin points to frame 0
    points, _ = cv.projectPoints(unitv_points, R_W0, T_W0, cmtx0, dist0)
    points = points.reshape((4,2)).astype(np.int32)
    origin = tuple(points[0])
    for col, _p in zip(colors, points[1:]):
        _p = tuple(_p.astype(np.int32))
        cv.line(frame0, origin, _p, col, 2)

    #project origin points to frame1
    R_W1 = R_01 @ R_W0
    T_W1 = R_01 @ T_W0 + T_01
    points, _ = cv.projectPoints(unitv_points, R_W1, T_W1, cmtx1, dist1)
    points = points.reshape((4,2)).astype(np.int32)
    origin = tuple(points[0])
    for col, _p in zip(colors, points[1:]):
        _p = tuple(_p.astype(np.int32))
        cv.line(frame1, origin, _p, col, 2)

    cv.imshow('frame0', frame0)
    cv.imshow('frame1', frame1)
    cv.waitKey(0)

    return R_W1, T_W1


def save_extrinsic_calibration_parameters(R0, T0, R1, T1, prefix = ''):
    
    #create folder if it does not exist
    if not os.path.exists('camera_parameters'):
        os.mkdir('camera_parameters')

    camera0_rot_trans_filename = os.path.join('camera_parameters', prefix + 'camera0_rot_trans.dat')
    outf = open(camera0_rot_trans_filename, 'w')

    outf.write('R:\n')
    for l in R0:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')

    outf.write('T:\n')
    for l in T0:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')
    outf.close()

    #R1 and T1 are just stereo calibration returned values
    camera1_rot_trans_filename = os.path.join('camera_parameters', prefix + 'camera1_rot_trans.dat')
    outf = open(camera1_rot_trans_filename, 'w')

    outf.write('R:\n')
    for l in R1:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')

    outf.write('T:\n')
    for l in T1:
        for en in l:
            outf.write(str(en) + ' ')
        outf.write('\n')
    outf.close()

    return R0, T0, R1, T1

def load_camera_intrinsics(camera_name):
    in_filename = os.path.join('camera_parameters',camera_name + '_intrinsics.dat')

    with open(in_filename, 'r') as inf:
        lines = inf.readlines()

    # Find the section markers
    intrinsic_start = lines.index('intrinsic:\n') + 1
    distortion_start = lines.index('distortion:\n')

    # Read camera matrix (assumes 3x3 matrix)
    camera_matrix = []
    for i in range(intrinsic_start, distortion_start):
        row = list(map(float, lines[i].strip().split()))
        camera_matrix.append(row)
    camera_matrix = np.array(camera_matrix)

    # Read distortion coefficients (assumes 1xN vector)
    distortion_coefs = list(map(float, lines[distortion_start + 1].strip().split()))
    distortion_coefs = np.array(distortion_coefs).reshape(1, -1)
    print(f"{camera_name=}\n {camera_matrix=}\n {distortion_coefs=}")

    return camera_matrix, distortion_coefs

if __name__ == '__main__':

    # if len(sys.argv) != 2:
    #     print('Call with settings filename: "python3 calibrate.py calibration_settings.yaml"')
    #     quit()
    bool_scratch = False

    #Open and parse the settings file
    parse_calibration_settings_file("calibration_settings.yaml")

    """Step1. Save calibration frames for single cameras"""
    
    save_frames_single_camera('camera0', 0) #save frames for camera0
    save_frames_single_camera('camera1', 1) #save frames for camera1
    print("cali saved")

    """Step2. Obtain camera intrinsic matrices and save them"""
    #camera0 intrinsics
    images_prefix = os.path.join('frames', 'camera0*')
    cmtx0, dist0 = calibrate_camera_for_intrinsic_parameters(images_prefix) 
    save_camera_intrinsics(cmtx0, dist0, 'camera0') #this will write cmtx and dist to disk
    #camera1 intrinsics
    images_prefix = os.path.join('frames', 'camera1*')
    cmtx1, dist1 = calibrate_camera_for_intrinsic_parameters(images_prefix)
    save_camera_intrinsics(cmtx1, dist1, 'camera1') #this will write cmtx and dist to disk

    #If start from the stereo calibration. -> comment our calibrarte_camera_for_intrinsic_parameters and save_camera_intrinsics
    #cmtx0, dist0 = load_camera_intrinsics('camera0')
    #cmtx1, dist1 = load_camera_intrinsics('camera1')

    """Step3. Save calibration frames for both cameras simultaneously"""
    save_frames_two_cams('camera0', 0, 'camera1', 1) #save simultaneous frames


    """Step4. Use paired calibration pattern frames to obtain camera0 to camera1 rotation and translation"""
    frames_prefix_c0 = os.path.join('frames_pair', 'camera0*')
    frames_prefix_c1 = os.path.join('frames_pair', 'camera1*')
    R, T = stereo_calibrate(cmtx0, dist0, cmtx1, dist1, frames_prefix_c0, frames_prefix_c1)


    """Step5. Save calibration data where camera0 defines the world space origin."""
    #camera0 rotation and translation is identity matrix and zeros vector
    R0 = np.eye(3, dtype=np.float32)
    T0 = np.array([0., 0., 0.]).reshape((3, 1))

    save_extrinsic_calibration_parameters(R0, T0, R, T) #this will write R and T to disk
    R1 = R; T1 = T #to avoid confusion, camera1 R and T are labeled R1 and T1
    #check your calibration makes sense
    camera0_data = [cmtx0, dist0, R0, T0]
    camera1_data = [cmtx1, dist1, R1, T1]
    check_calibration('camera0', 0, camera0_data, 'camera1', 1, camera1_data, _zshift = 60.)


    """Optional. Define a different origin point and save the calibration data"""
    # #get the world to camera0 rotation and translation
    # R_W0, T_W0 = get_world_space_origin(cmtx0, dist0, os.path.join('frames_pair', 'camera0_4.png'))
    # #get rotation and translation from world directly to camera1
    # R_W1, T_W1 = get_cam1_to_world_transforms(cmtx0, dist0, R_W0, T_W0,
    #                                           cmtx1, dist1, R1, T1,
    #                                           os.path.join('frames_pair', 'camera0_4.png'),
    #                                           os.path.join('frames_pair', 'camera1_4.png'),)

    # #save rotation and translation parameters to disk
    # save_extrinsic_calibration_parameters(R_W0, T_W0, R_W1, T_W1, prefix = 'world_to_') #this will write R and T to disk