import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import numpy as np
from PIL import Image
import os
import json
import copy
import pandas as pd
from statistics import stdev, mean
import time
from bop_toolkit.bop_toolkit_lib import pose_error,misc
from icp_utils import dpt_2_cld, prepare_dataset, execute_global_registration

rgb_format = '.png'
dpt_format = '.png'
seg_format = '.npy'

class this_dataset():
    
    def __init__(self, stage, dataset):

        self.rt_dir = dataset
        self.base_dir= dataset+'/'+stage
        self.rgb_dir = dataset+'/'+stage+'/rgb'
        self.dpt_dir = dataset+'/'+stage+'/depth'
        self.seg_dir = dataset+'/'+stage+'/seg_maps'
        self.mesh_path = dataset +'/model_meshes/'
        self.pcld_path = dataset +'/model_pointcloud/'

    def get_item(self, im_id):
        
        #1. Read RGB frame and depth frame
        with Image.open(os.path.join(self.rgb_dir,str(im_id)+ rgb_format)) as ri:
            rgb = np.array(ri,dtype=np.uint8)[:, :, :3]
        
        
        with Image.open(os.path.join(self.dpt_dir,str(im_id)+ dpt_format)) as di:
            dpt = np.array(di)
            
        #2. Read segmentation map
        segmap = np.load(os.path.join(self.seg_dir,str(im_id)+'_seg_map'+seg_format))
        
        #3. Read scene camera ground truth
        with open(os.path.join(self.base_dir,'scene_camera.json'),"r") as k:
            for i,j in enumerate(k):
                im_dict = json.loads(j)
                if i == int(im_id):
                    this_cam = im_dict

        #4. Read scene ground truth wrt camera frame
        with open(os.path.join(self.base_dir,'scene_gt.json'),"r") as l:
            for i,j in enumerate(l):
                scene_dict = json.loads(j)
                if i == int(im_id):
                    this_scene = scene_dict
        
        #5 Pose relative to base object           
        with open(os.path.join(self.rt_dir,'gt_assembly_poses.json'),"r") as ap:
            pose_dict = json.load(ap)
            
        with open(os.path.join(self.rt_dir,'models_info.json'),"r") as m:
            models_info = json.load(m)
            
        with open(os.path.join(self.base_dir,'scene_w_gt.json'),"r") as n:
            for i,j in enumerate(n):
                scene_dict = json.loads(j)
                if i == int(im_id):
                    this_scene_w = scene_dict
        
        return rgb, dpt, segmap, this_cam[im_id], pose_dict, this_scene_w[im_id], models_info, this_scene[im_id]

def get_target_pcd(stage, cld ,this_cam):
    
    target_pcd_wrt_opt = o3d.geometry.PointCloud()
    target_pcd_wrt_opt.points = o3d.utility.Vector3dVector(cld)
    
    # tranformation matrix of depth_optical_frame wrt to world_frame
    dptopt_R_w2c = np.array(this_cam['dptopt_cam_R_w2c']).reshape(3,3)
    dptopt_t_w2c = np.array(this_cam['dptopt_cam_t_w2c']).reshape(3,1)*0.001
    dptopt_world_T = np.hstack((dptopt_R_w2c,dptopt_t_w2c))
    dptopt_world_T = np.vstack((dptopt_world_T, [0,0,0,1]))
    

    target_pcd_wrt_world = target_pcd_wrt_opt.transform(dptopt_world_T)
 
    return target_pcd_wrt_world, dptopt_world_T

def pcl_registration(target, source_mesh_,dpt_K, dptopt_world_T):
    
    cam_position = np.transpose(dptopt_world_T[:3,3])    
    source_mesh = copy.deepcopy(source_mesh_)
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(source_mesh)
    mesh.translate(target.get_center())
    
    initial_T = np.hstack((np.eye(3,3),np.array(target.get_center()).reshape(3,1)))
    initial_T = np.vstack((initial_T, [0,0,0,1]))                              
    
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh)

    # https://learnwebgl.brown37.net/07_cameras/camera_introduction.html
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
                            fov_deg = 60,
                            center= target.get_center(),
                            eye= cam_position,
                            up=[0, 1, 0],
                            width_px=1280,
                            height_px=720,)

    ans = scene.cast_rays(rays)    
    hit = ans['t_hit'].isfinite()
    points = rays[hit][:,:3] + rays[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))
    source_pcd = o3d.t.geometry.PointCloud(points)
    source_pcd = o3d.t.geometry.PointCloud.to_legacy(source_pcd).paint_uniform_color([0, 0.651, 0.929])
    
    pcd_world_T = np.hstack((np.eye(3,3),np.array(source_pcd.get_center()).reshape(3,1)))
    pcd_world_T = np.vstack((pcd_world_T, [0,0,0,1]))

    mesh_world_T = np.hstack((np.eye(3,3),np.array(target.get_center()).reshape(3,1)))
    mesh_world_T = np.vstack((mesh_world_T, [0,0,0,1]))

    mesh_pcd_T = np.dot(np.linalg.inv(pcd_world_T), mesh_world_T)
    
    voxel_size = 0.008  # 0.03
    radius_normal = voxel_size*2
    radius_feature = voxel_size *3
    global_distance_threshold = voxel_size * 1.2
    local_distance_threshold = voxel_size * 0.4
    
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, 
                                                                    source_pcd,target, radius_normal,radius_feature)

    #1. Global registration    
    result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh, voxel_size,
                                            global_distance_threshold)

    #2. Local registration
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, local_distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    
    estimated_base_T = np.dot(mesh_pcd_T,np.dot(reg_p2p.transformation,pcd_world_T))
    
    return target,source, reg_p2p ,result_ransac, estimated_base_T

def eval_assembly_pose(item,stage,models,im_id):
  
    rgb,dpt,seg_map,this_cam, pose_dict,this_scene_w, models_info, this_scene = item.get_item(im_id)
    
    cam_K = this_cam['cam_K']
    K = np.array(cam_K).reshape(3,3)
    dpt_cam_K = this_cam['dpt_cam_K']
    dpt_K = np.array(dpt_cam_K).reshape(3,3)
    
    seg_mask = np.zeros_like(seg_map)
    
    for i in range (1,stage+1):
            this_seg_mask = copy.deepcopy(seg_map)
            this_seg_mask[np.where(this_seg_mask != i)] = 0
            this_seg_mask[np.where(this_seg_mask != 0)] = 1
            seg_mask += this_seg_mask 
        
    cld,_ = dpt_2_cld(dpt,dpt_K,seg_mask)
    
    #1.get target pointcloud <== the pointcloud inferred from camera
    target_pcd_wrt_world, dptopt_world_T = get_target_pcd(stage, cld ,this_cam)
    
    #2.get source mesh
    source_mesh = o3d.io.read_triangle_mesh(item.rt_dir+'/model_meshes/stage_'+str(stage)+'.ply')
    
    #3.do registration
    target, source,reg_result,result_ransac, estimated_base_T = pcl_registration(target_pcd_wrt_world, source_mesh, dpt_K, dptopt_world_T)
  
    # ground-truth assembly pose wrt base 
    R_wrt_base = np.asarray(pose_dict[str(stage)]["R_wrt_bottom_casing"]).reshape(3,3)
    t_wrt_base = np.asarray(pose_dict[str(stage)]["t_wrt_bottom_casing"]).reshape(3,1)*0.001 # already in mm
    obj_base_T = np.hstack((R_wrt_base,t_wrt_base))
    obj_base_T = np.vstack((obj_base_T, [0,0,0,1]))
    
    estimated_assembly_pose = np.dot(estimated_base_T,obj_base_T)
    
    est_R_world = estimated_assembly_pose[0:3,0:3]
    est_t_world = estimated_assembly_pose[:3, 3].reshape(3,1)
    
    #(this is unnecesssary read gt pose from next stage!)
    # ground_truth assembly pose wrt world ==> base_world_T * obj_base_T
    gt_R_world =  np.array(this_scene_w[0]["cam_R_m2c"]).reshape(3,3) ## cam_R_m2c must be changed in gen script!!!
    gt_t_world =  np.array(this_scene_w[0]["cam_t_m2c"]).reshape(3,1)*0.001
    gt_base_world_T = np.hstack((gt_R_world,gt_t_world))
    gt_base_world_T = np.vstack((gt_base_world_T, [0,0,0,1]))
    
    gt_assembly_pose = np.dot(gt_base_world_T,obj_base_T)
    gt_R_world = gt_assembly_pose[0:3,0:3]
    gt_t_world = gt_assembly_pose[:3, 3].reshape(3,1)
    
    
    assembling_pcd = o3d.io.read_point_cloud(item.rt_dir+'/model_pointcloud/'+models[stage]+'.ply')
    pts = np.asarray(assembling_pcd.points)
    
    syms = misc.get_symmetry_transformations(models_info[str(stage)], max(0.015, models_info[str(stage)]['diameter']))

    
    error = pose_error.adi(est_R_world, est_t_world, gt_R_world, gt_t_world, pts)
    translation_error = pose_error.te(est_t_world,gt_t_world)
    rotational_error = pose_error.re(est_R_world,gt_R_world)
    # https://bop.felk.cvut.cz/challenges/bop-challenge-2019/#:~:text=7.-,Evaluation%20methodology,-7.1%20Pose%2Derror
    mssd_error = pose_error.mssd(est_R_world, est_t_world, gt_R_world, gt_t_world, pts, syms)

    return reg_result , error, translation_error,rotational_error , mssd_error , gt_assembly_pose ,  estimated_assembly_pose

if __name__ == "__main__":
    
    data_all, data_all_summary = {}, {}
    inliers, fitness_vals, ADI_error, translation_error, rotational_error, mssd_error, inference_time = [], [], [], [], [], [], []
    
    # Uncomment the dataset to test

    ## Nema17 dataset ############
    dataset = "Nema17_reducer_dataset"
    models = ['Nema17','sun_gear','housing','carrier','cover']

    ## Fidget dataset ############
    #dataset = "fidget_dataset"
    #models = ["bottom_casing", "left_gear", "right_gear", "top_casing"]
    
    stage = 1 # replace this with assembly stage
    n_instances = 400 # replace this with number of images in each stage

    for i in range(0, n_instances):
        time.sleep(3)
        start = time.time()
        if i% 100 == 0:
            print("calculating instance :", str(i))

        item = this_dataset(stage,dataset)
        reg_result, error,te,re, mssd, gt_assembly_pose = eval_assembly_pose(item,stage,models,im_id=str(i))
        
        if reg_result.fitness < 0.6:
            print("fitness too low :",reg_result.fitness)
            print("calculating instance :", str(i))

        end = time.time()
        run_time = (end-start) # time in milli seeconds
        
        inference_time.append(run_time)
        inliers.append(reg_result.inlier_rmse)
        fitness_vals.append(reg_result.fitness)
        ADI_error.append(error)
        translation_error.append(te)
        rotational_error.append(re)
        mssd_error.append(mssd)

    stats ={"inliers" :inliers,
            "fitness": fitness_vals,
            "ADI error":  ADI_error,
            "translation_error": translation_error,
            "rotational_error": rotational_error,
            "MSSD_error": mssd_error,
            "inference_time":inference_time}
    
    stats_summary = {
            "stage": ["stage_"+str(stage)],
            "fitness_mean": [float(mean(fitness_vals))],
            "fitness_stdv": [stdev(fitness_vals)],
            "inliers_mean": [float(mean(inliers))],
            "inliers_stdv": [stdev(inliers)],
            "ADI_mean": [mean(ADI_error)],
            "ADI_stdv": [stdev(ADI_error)],
            "MSSD_mean": [mean(mssd_error)],
            "MSSd_stdv": [stdev(mssd_error)],
            "mean_time":[mean(inference_time)]}
   
    print("done")
    print("results :", stats_summary)
    
    #save to pkl files
    data = pd.DataFrame(stats)
    data_all = {"stage_"+str(stage):data}

    data_summary = pd.DataFrame(stats_summary)
    data_all_summary = {"stage_"+str(stage):data_summary}

    data.to_pickle(dataset+"/"+ "stage_"+str(stage)+".pkl")
    data_summary.to_pickle(dataset+"/"+ "stage_"+str(stage)+"_summary.pkl") 