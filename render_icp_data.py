#%% Importing libraries
import sys
import numpy as np
import open3d as o3d
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import LinkState as stateGZ
from gazebo_msgs.srv import GetModelState as getStateGZ
from gazebo_msgs.srv import SetLinkState as setStateGZ
from geometry_msgs.msg import (Point, Pose, PoseArray, PoseStamped, Quaternion,
                               Twist, Vector3)
from rospy.exceptions import ROSException
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import copy
import time
import cv2
import scipy.io as sio
import os
import argparse
import json
from numpy import save



# cd ~/tf2_tools  &&  source devel/setup.bash
# rosrun tf2_tools echo2.py base_link camera_depth_optical_frame

# python3 render_icp_data.py -m bottom_casing left_gear right_gear top_casing -d fidget -s 1
# python3 render_icp_data.py -m Nema17 sun_gear housing carrier cover -d Nema17_reducer -s 1
# python3 render_icp_data.py -mcase cover gear_0 gear_1 gear_1_g_testing gear_carrier -d Planetary_reducer -s 1

#%% Pass the models to be rendered as arguments via argument parser
parser = argparse.ArgumentParser(description = 'parse some parameters')
parser.add_argument('-m','--models', nargs='+', help="Enter the names of the models seperated by a space",required=True)
parser.add_argument('-d','--dataset',type=str, help="enter dataset name",required=True)
parser.add_argument('-s','--stage' ,type=int, help="enter the stage of assembly",required=True)
#args = parser.parse_args('-m bottom_casing left_gear right_gear top_casing -d fidget_dataset -s 1'.split())
args = parser.parse_args()

n_models = len(args.models)
dataset_name = args.dataset
stage_number = str(args.stage)

#%% Create the directories if they don't already exist
if not os.path.exists(dataset_name+'_dataset/stage_'+stage_number+'/rgb'):
    os.makedirs(dataset_name+'_dataset/stage_'+stage_number+'/rgb')
if not os.path.exists(dataset_name+'_dataset/stage_'+stage_number+'/depth'):
    os.makedirs(dataset_name+'_dataset/stage_'+stage_number+'/depth')
if not os.path.exists(dataset_name+'_dataset/stage_'+stage_number+'/seg_maps'):
    os.makedirs(dataset_name+'_dataset/stage_'+stage_number+'/seg_maps')
if not os.path.exists(dataset_name+'_dataset/stage_'+stage_number+'/masks'):
    os.makedirs(dataset_name+'_dataset/stage_'+stage_number+'/masks')
if not os.path.exists(dataset_name+'_dataset/model_pointcloud'):
    os.makedirs(dataset_name+'_dataset/model_pointcloud')
if not os.path.exists(dataset_name+'_dataset/model_meshes'):
    os.makedirs(dataset_name+'_dataset/model_meshes')  

print('Generating data for '+str(n_models)+' selected models')
#%% Initialize ros
rospy.init_node('data_render_gazebo', anonymous = True)
rate = rospy.Rate(0.5)
bridge = CvBridge()
cam_info_msg = rospy.wait_for_message('camera/color/camera_info', CameraInfo, timeout = 2)
dpt_cam_info_msg = rospy.wait_for_message('camera/depth/camera_info', CameraInfo, timeout = 2)


#%% Set camera pose in gazebo
def set_cam_state_gazebo(camPos, camTwist):
    # Set cam state in gazebo
    camstate = stateGZ('robot::base_link', camPos, camTwist, 'world' )
    print('Transforming camera to pose : '+str(sample_num))
    try:
       gzclient = rospy.ServiceProxy('gazebo/set_link_state', setStateGZ)
       resp = gzclient(camstate)
        
    except Exception as inst:
           print('Error in gazebo/set_link_state service request: ' + str(inst) )

#%% Calculate parameters for current loop for camera state setting
def calc_params(phi,theta,dist):
    theta_rad = np.deg2rad(theta)
    phi_rad = np.deg2rad(phi)
    X = dist*np.cos(phi_rad)*np.cos(theta_rad)
    Y = dist*np.cos(phi_rad)*np.sin(theta_rad)
    Z = np.abs(dist*np.sin(phi_rad)) + 0.84

    cam_euler = R.from_euler('xyz',[0,phi,theta+180], degrees=True)
    cam_quat = cam_euler.as_quat()
    
    camPos = Pose(position= Point(x=X, y=Y, z=Z), 
                  orientation= Quaternion(x=cam_quat[0], y=cam_quat[1] , z=cam_quat[2], w=cam_quat[3]))
    camTwist = Twist(linear= Vector3(x=0, y=0, z=0) , 
                     angular= Vector3(x=0, y=0, z=0))
    
    return camPos,camTwist, X, Y,Z, cam_euler 

#%% inplane rotation of the kinect      

 #%% Function to convert between Image types for depth images
def convert_types(img, orig_min, orig_max, tgt_min, tgt_max, tgt_type):

    #info = np.finfo(img.dtype) # Get the information of the incoming image type
    # normalize the data to 0 - 1
    img_out = img / (orig_max-orig_min)   # Normalize by input range
    img_out = (tgt_max - tgt_min) * img_out # Now scale by the output range
    img_out = img_out.astype(tgt_type)

    #cv2.imshow("Window", img)
    return img_out

#%% Avoid duplicates
def check_dup():
    rgb_duplicate = True                     
    while rgb_duplicate:
        print('Subscribing to rgb topics...')
        img_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout = 3)
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if sample_num > 0:  
            #No point checking for the sample 0
            previous_im = cv2.imread(dataset_name+'_dataset/stage_'+stage_number+'/rgb/'+str(sample_num-1)+'.png', -1)
            rgb_duplicate = abs(np.mean(cv_image - previous_im)) < 0.5 # Mean of all pixels shouldn't be this small if it's two different images
            print('rgb diff: '+str(np.mean(cv_image - previous_im)))
            print(rgb_duplicate)
            if rgb_duplicate:
                #Try setting state again. Sometimes gazebo trips out as well.
                set_cam_state_gazebo(camPos, camTwist)

        else:
            rgb_duplicate = False

    depth_duplicate = True 
    while depth_duplicate:
        print('Subscribing to depth topics...')
        depthImg_msg = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout = 3 )
        cv_depthImage = bridge.imgmsg_to_cv2(depthImg_msg, desired_encoding='passthrough')
        if sample_num > 0:
            previous_im = cv2.imread(dataset_name+'_dataset/stage_'+stage_number+'/depth/'+str(sample_num-1)+'.png', -1)
            depth_duplicate = abs(np.nanmean(cv_depthImage - previous_im))< 50  # Mean of all pixels shouldn't be this small if it's two different images
            print('depth diff: '+str(np.nanmean(cv_depthImage - previous_im)))# - previous_im)))
            print(depth_duplicate)
            if depth_duplicate:
                #Try setting state again. Sometimes gazebo trips out as well.
                set_cam_state_gazebo(camPos, camTwist)
        else:
            depth_duplicate = False
          
    return cv_image, cv_depthImage

#%% Get Camera Extrinsics
def get_camera_extrinsics(phi,theta, dist):
    
    _,_,X,Y,Z,_ = calc_params(phi,theta,dist)

    cam_euler = R.from_euler('xyz',[0,phi,theta+180], degrees=True)
    cam_world_R = cam_euler.as_matrix()
    cam_world_t = np.array([X,Y,Z]).reshape(3,1)
    cam_world_T = np.hstack((cam_world_R,cam_world_t))
    cam_world_T = np.vstack((cam_world_T, [0,0,0,1]))
    
    return cam_world_T

#%% get object states and save pose
def get_object_states(n_models):
    resp=[]
    #Get object state 
    try: 
        rospy.wait_for_service('gazebo/get_model_state')
        client = rospy.ServiceProxy('gazebo/get_model_state', getStateGZ)
        for i in range(n_models):
            #print(args.models[i])
            #print(client(args.models[i], 'world'))
            resp.append( client(args.models[i], 'world'))
    except Exception as inst:
        print('Error in gazebo/get_link_state service request: ' + str(inst) )
    return resp

#%% True Object pose in world frame obtained from Gazebo Service
def get_object2cam_pose(resp, n_models, cam_world_T):
    
    obj_cam_T = np.zeros(( n_models, 4, 4)) # Transformation Mats for 10 object classes
    obj_cam = np.zeros(( n_models,4, 4))
    obj_world = np.zeros((n_models, 4, 4))
    
    
    for i in range(0 , n_models):
        obj_pos = np.array([resp[i].pose.position.x, resp[i].pose.position.y,resp[i].pose.position.z]).reshape(3,1)
        obj_or = [resp[i].pose.orientation.x, resp[i].pose.orientation.y, resp[i].pose.orientation.z, resp[i].pose.orientation.w]
        obj_or = (R.from_quat(obj_or)).as_matrix()
        obj_world_T = np.concatenate((obj_or, obj_pos), axis = 1)
        

        # Transformation from object2world to  object2cam for GT label poses
        #obj_cam_T = np.dot(obj_world_T, np.linalg.inv(cam_world_T) )
        obj_world_T = np.vstack(( obj_world_T, [0,0,0,1] ))
        
        obj_world[i,:,:] = obj_world_T   
        
        obj_cam_T[i, :, :] = np.dot( np.linalg.inv(cam_world_T), obj_world_T )#[:3,:]
        obj_cam[i,:,:] = np.dot( np.linalg.inv(cam_world_T), obj_world_T )
        #print(i,np.dot( np.linalg.inv(cam_world_T), obj_world_T ))
        
    #print(obj_cam_T)
    gt_dict = { 'poses':obj_cam_T[:3,:,:] } #returns [ R  T , i] 
    return  gt_dict, obj_cam_T, obj_cam, obj_world

#%% Load the meshes of all objects convert them to point clouds
# combine and return the pointclouds of all meshes in a dictionary
def mesh2pcld(n_models):

    all_points = {}
    all_pclds  = {}
    mesh_path = dataset_name+'_dataset/model_meshes/'
    pcld_path = dataset_name+'_dataset/model_pointcloud/'
    
    for i in range (0,n_models):
        
        model = str(args.models[i])
        if not os.path.exists(pcld_path + model +'.ply'):
        	
        
            mesh = o3d.io.read_triangle_mesh(mesh_path + str(args.models[i])+'.ply')
            pcld = mesh.sample_points_poisson_disk(number_of_points=30000)
            all_pclds[model] = pcld
            o3d.io.write_point_cloud(pcld_path + model +'.ply', pcld )
            all_points[model] = np.asarray(pcld.points)#, dtype= np.float32)
	    
        else:
            print( model +" pointcloud file already exists..")
            pcld = o3d.io.read_point_cloud (pcld_path + model + '.ply')
            all_pclds[model] = pcld
            all_points[model] = np.asarray(pcld.points)
                                
    return all_points, all_pclds

#%% Function to fill empty spaces in point cloud project
def cv_im_fill(bin_im):
    im_floodfill = bin_im.copy()
    # Mask used to flood filling.
    # Notice the size needs to be 2 pixels smaller than the image.
    h, w = bin_im.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    # Floodfill from point (0, 0)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
    # Invert floodfilled image
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    # Combine the two images to get the foreground.
    im_out = bin_im | im_floodfill_inv
    return im_out

def get_pcd_order(all_pclds,obj_world, models_list,dpt_opt_world_T):
    
    pclds_centers =[]
    for i in range(0, len(models_list)) :
        this_pcld = all_pclds[str(models_list[i])]
        transformation_in_opt_frame = np.dot(np.linalg.inv(dpt_opt_world_T), obj_world[i,:,:])
        pcd_in_opt_frame = copy.deepcopy(this_pcld).transform(transformation_in_opt_frame)
        pclds_centers.append(-pcd_in_opt_frame.get_center()[2])
    
    pclds_from_farthest_2_closest = np.argsort(pclds_centers)
    
    return pclds_from_farthest_2_closest

#%% Transform the pointclouds to binary mask
def pcl_2_binary_mask(obj_cam_T,n_models, all_points):
    
    #Projection Matrix / Camera instrinsics
    cam_P = np.array(cam_info_msg.P).reshape(3,4)
    cam_P = np.vstack((cam_P , [0,0,0,1]))
    dpt_cam_P = np.array(dpt_cam_info_msg.P).reshape(3,4)
    dpt_cam_P = np.vstack((dpt_cam_P , [0,0,0,1]))


    # Empty arrays
    bin_mask = np.zeros((cam_info_msg.height, cam_info_msg.width), dtype= np.uint8)
    seg_mask  = np.zeros((cam_info_msg.height, cam_info_msg.width), dtype= np.uint8)
    mask_list = []
    pixels_list = []


    # get the tranformation matrix for depth optical_frame wrt to base_link frame
    dpt_opt_cam_T = rospy.wait_for_message('/Cam_2_World', PoseStamped )
    T_pos = np.array([dpt_opt_cam_T.pose.position.x, dpt_opt_cam_T.pose.position.y, dpt_opt_cam_T.pose.position.z])
    T_pos = T_pos.reshape(3,1)
    T_or = [dpt_opt_cam_T.pose.orientation.x, dpt_opt_cam_T.pose.orientation.y, dpt_opt_cam_T.pose.orientation.z, dpt_opt_cam_T.pose.orientation.w]
    T_or = (R.from_quat(T_or)).as_matrix()
    dpt_opt_cam_T = np.concatenate((T_or, T_pos), axis = 1) 
    dpt_opt_cam_T = np.vstack((dpt_opt_cam_T, [0,0,0,1] ))

    # get the tranformation matrix for depth_optical_frame wrt to world_frame
    dpt_opt_world_T = np.dot(cam_world_T, dpt_opt_cam_T)


    # get pclds ordered in farthest to closest distance
    pcd_order = get_pcd_order(all_pclds,obj_world, args.models,dpt_opt_world_T)

    for i in pcd_order:
        print(str(args.models[i]))
        print(i)
        
        #1. Get the pose of this_object in optical frame
        thisObj_world_T = obj_world[i,:,:]
        thisObj_opt_T = np.dot(np.linalg.inv(dpt_opt_world_T), thisObj_world_T)

        #1. Transform the pointcloud of this_object to optical frame
        this_pcld = all_pclds[str(args.models[i])]
        cloud_optical = copy.deepcopy(this_pcld).transform(thisObj_opt_T)
        cloud_optical = np.asarray(cloud_optical.points).transpose()
        cloud_optical = np.vstack((cloud_optical, np.ones((1,cloud_optical.shape[1])) ))
        
        
        # perspective projection into image-plane
        x,y,z,w = np.dot( dpt_cam_P,cloud_optical).astype(np.float32) #This is the projection step
        x = x / z
        y = y / z


        #clips out all the points projected out of image height and width
        clipping = np.logical_and( np.logical_and(x>=0, x<=cam_info_msg.width) , 
                                np.logical_and(y>=0, y<=cam_info_msg.height) )
        x = x[np.where(clipping)]
        y = y[np.where(clipping)]


        #Leave the background black
        pixels = np.vstack((x,y)).transpose()
        pixels = np.array(pixels, dtype=np.uint16)
        pixels_list.append([pixels])

        this_mask = np.zeros((cam_info_msg.height, cam_info_msg.width), dtype= np.uint8)

        for point in pixels:
            this_mask[point[1]-1, point[0]-1] = 255

        this_mask = cv_im_fill(this_mask)
        
        this_mask[this_mask.nonzero()] = 1.05*np.ceil(255*(i+1)/n_models)
        r,c = this_mask.nonzero()
        
        bin_mask[this_mask.nonzero()] = 0
        bin_mask += this_mask
        
        seg_mask[this_mask.nonzero()] = i+1

           
    return bin_mask, seg_mask , dpt_opt_world_T

#%% Main program

# convert the meshes to pointclouds 
print('Generating the pointclouds for '+str(n_models)+ ' models. hold on , This could take few minutes ...')
all_points, all_pclds = mesh2pcld(n_models)
print(all_points.keys())
sample_num = 0
#0.15,0.5,0.125
#0.25,0.6,0.0625
for dist in np.arange(0.25,0.6,0.0625):
    for phi in range(35,70,15):
        for theta in range(0, 360, 15):
        
            camPos,camTwist,_,_,_,_ =  calc_params(phi,theta,dist)
            set_cam_state_gazebo(camPos, camTwist)
            
            
            while not rospy.is_shutdown():
                print('Subscribing to camera topics...')
                try:
                    cv_image, cv_depthImage = check_dup()
                    break  
                except ROSException as e:
                        print('Timeout occured in subscribing.Trying again...')
                        continue
            
            #cv_depthImage = convert_types(cv_depthImage,0,3, 0,65535, np.uint16) ## 0 - 3m is the input range of kinect depth
            print('Writing Images')
            cv2.imwrite(dataset_name+'_dataset/stage_'+stage_number+'/rgb/'+str(sample_num)+'.png', cv_image)
            cv2.imwrite(dataset_name+'_dataset/stage_'+stage_number+'/depth/'+str(sample_num)+'.png',cv_depthImage)
                    
            try:
                resp = get_object_states(n_models)
            except Exception as inst:
                     print('Error in gazebo/get_link_state service request: ' + str(inst) )
                
            cam_world_T = get_camera_extrinsics(phi,theta,dist)
            gt_dict,obj_cam_T,obj_cam, obj_world = get_object2cam_pose(resp, n_models,cam_world_T)
            
        
            # save binary mask and segmentation map
            bin_mask, seg_mask, dpt_opt_world_T = pcl_2_binary_mask(obj_cam_T, n_models, all_points)
            
            print('Writing binary mask images') 
            cv2.imwrite(dataset_name+'_dataset/stage_'+stage_number+'/masks/'+str(sample_num)+'.png',bin_mask)
            
            print('Saving segment maps as numpy files')
            save(dataset_name+'_dataset/stage_'+stage_number+'/seg_maps/'+str(sample_num)+'_seg_map.npy', seg_mask)
            
            # save scene_gt and scene_camera
            print('Writing scene_gt,scene_w_gt,scene_camera json files', '\n'*5)
            
            with open(dataset_name+'_dataset/stage_'+stage_number+"/scene_gt.json", "a") as outfile:
                gt_all= {}
                obj_list =[]
                for i in range(0,n_models):
                    obj_data = {
                        "obj_id": str(i),
                        "cam_R_m2c": np.ravel(np.reshape(obj_cam[i][0:3,0:3],(1,9))).tolist(),
                        "cam_t_m2c": (obj_cam[i][0:3,3]*1000).tolist(),                 
                    }
                    obj_list.append(obj_data)
                gt_all[str(sample_num)] = obj_list
                json.dump(gt_all, outfile)
                outfile.write("\n") # Add newline cause Py JSON does not
            
            with open(dataset_name+'_dataset/stage_'+stage_number+"/scene_w_gt.json", "a") as outfile:
                gt_w_all= {}
                obj_list =[]
                for i in range(0,n_models):
                    obj_data = {
                        "obj_id": str(i),
                        "cam_R_m2c": np.ravel(np.reshape(obj_world[i][0:3,0:3],(1,9))).tolist(),
                        "cam_t_m2c": (obj_world[i][0:3,3]*1000).tolist()                  
                    }
                    obj_list.append(obj_data)
                gt_w_all[str(sample_num)] = obj_list
                json.dump(gt_w_all, outfile)
                outfile.write("\n") # Add newline cause Py JSON does not
			    
            with open(dataset_name+'_dataset/stage_'+stage_number+"/scene_camera.json", "a") as outfile:
                    scene_cam = {}
                    obj_data = {
                        "dpt_cam_K": np.array(dpt_cam_info_msg.K).tolist(),
                        "cam_K" : np.array(cam_info_msg.K).tolist() ,
                        "depth_scale" : 1.0,
                        "dptopt_cam_R_w2c":np.ravel(np.reshape(dpt_opt_world_T[0:3,0:3],(1,9))).tolist() ,
                        "dptopt_cam_t_w2c": (dpt_opt_world_T[0:3,3]*1000).tolist(),
                        "cam_R_w2c":np.ravel(np.reshape(cam_world_T[0:3,0:3],(1,9))).tolist() ,
                        "cam_t_w2c": (cam_world_T[0:3,3]*1000).tolist(),    
                    }
                    
                    #this_cam.append(obj_data) 
                    scene_cam[str(sample_num)] = obj_data
                    json.dump(scene_cam, outfile)
                    outfile.write("\n") # Add newline cause Py JSON does not
            sample_num += 1


