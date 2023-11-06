import open3d as o3d
import numpy as np
import copy
import cv2

# Portions of this code have been adapted from the Open3D library (https://github.com/intel-isl/Open3D)
# and the PVN3D GitHub repository (https://github.com/ethnhe/PVN3D).
# Credit and thanks to the original authors for their contributions.

def prepare_dataset(voxel_size, source,target, radius_normal,radius_feature):
    #print(":: Load two point clouds and disturb initial pose.")
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size, radius_normal,radius_feature)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size,radius_normal,radius_feature)
    
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def preprocess_point_cloud(pcd, voxel_size, radius_normal,radius_feature ):
    
    pcd_down = pcd
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size, distance_threshold):
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        5, 
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.09),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], 
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.899))
    return result


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    target_temp.paint_uniform_color([1, 0.706, 0])
    source_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
                          
def draw_bounding_box(img, corner, gt=False):
    
    if gt:
        color = (255, 0, 0)
    else:
        color = (0, 255, 0)

    img = cv2.line(img, tuple(corner[0]), tuple(corner[1]), color, 2)
    img = cv2.line(img, tuple(corner[0]), tuple(corner[2]), color,2)
    img = cv2.line(img, tuple(corner[2]), tuple(corner[3]), color, 2)
    img = cv2.line(img, tuple(corner[3]), tuple(corner[1]), color, 2)
    img = cv2.line(img, tuple(corner[4]), tuple(corner[5]), color, 2)
    img = cv2.line(img, tuple(corner[5]), tuple(corner[7]), color, 2)
    img = cv2.line(img, tuple(corner[6]), tuple(corner[2]), color, 2)
    img = cv2.line(img, tuple(corner[3]), tuple(corner[7]), color, 2)
    img = cv2.line(img, tuple(corner[6]), tuple(corner[2]), color, 2)
    img = cv2.line(img, tuple(corner[4]), tuple(corner[0]), color, 2)
    img = cv2.line(img, tuple(corner[4]), tuple(corner[6]), color, 2)
    img = cv2.line(img, tuple(corner[6]), tuple(corner[7]), color, 2)
    img = cv2.line(img, tuple(corner[1]), tuple(corner[5]), color, 2)
   

    return img

def project_p3d( p3d, cam_scale, K):
    if p3d.shape[1]<4:
        p3d = p3d * cam_scale
        p2d = np.dot(p3d, K.T)
        p2d_3 = p2d[:, 2]
        p2d_3[np.where(p2d_3 < 1e-8)] = 1.0
        p2d[:, 2] = p2d_3
        p2d = np.around((p2d[:, :2] / p2d[:, 2:])).astype(np.int32)
        return p2d
    else:
        p3d = p3d * cam_scale
        p2d = np.dot(p3d[: , 0:3], K.T)
        p2d_3 = p2d[:, 2]
        filter = np.where(p2d_3 < 1e-8)
        if filter[0].shape[0]>0:
            p2d_rgbs = p3d[filter, 3:6]
            p2d_3[filter] = 1.0
        else:
            p2d_rgbs = p3d[:, 3:6]
        p2d[:, 2] = p2d_3
        p2d = np.around((p2d[:, :2] / p2d[:, 2:])).astype(np.int32)

        return np.concatenate((p2d, p2d_rgbs), axis=1).astype(np.int32)

#Function to project depth to pointcloud
def dpt_2_cld( depth_frame, K,segMask = None,cam_scale=1):

    w = depth_frame.shape[1]
    h = depth_frame.shape[0]
    
    xmap = np.array([[j for i in range(w)] for j in range(h)])
    ymap = np.array([[i for i in range(w)] for j in range(h)])
    
    dpt = np.array(depth_frame, dtype=np.float32)
    dpt = dpt/1000
    
    if len(dpt.shape) > 2:
            dpt = dpt[:, :, 0]
    msk_dp = dpt > -1
    choose = msk_dp.flatten().nonzero()[0].astype(np.uint32)

    if len(choose) < 1:
        return None, None
        
    dpt_mskd = dpt.flatten()[choose][:, np.newaxis].astype(np.float32)
    xmap_mskd = xmap.flatten()[choose][:, np.newaxis].astype(np.float32)
    ymap_mskd = ymap.flatten()[choose][:, np.newaxis].astype(np.float32)
    cam_cx, cam_cy = K[0][2], K[1][2]
    cam_fx, cam_fy = K[0][0], K[1][1]
    
    if segMask is not None:
             
        focus_points = np.argwhere(segMask != 0)
        focus = segMask != 0
        
        # projecting only the focus
        pt2 = dpt_mskd[focus.flatten()] / cam_scale
        pt0b= (ymap_mskd[focus.flatten()] - cam_cx) * pt2 / cam_fx
        pt1b= (xmap_mskd[focus.flatten()] - cam_cy) * pt2 / cam_fy
        focus_points = np.concatenate((pt0b, pt1b, pt2),axis=1)
               
        return focus_points , choose
    
    else :

        # projecting the cloud as a whole    
        pt2 = dpt_mskd / cam_scale
        pt0 = (ymap_mskd - cam_cx) * pt2 / cam_fx
        pt1 = (xmap_mskd - cam_cy) * pt2 / cam_fy
        cld = np.concatenate((pt0, pt1, pt2),axis=1)
        
        return cld , choose