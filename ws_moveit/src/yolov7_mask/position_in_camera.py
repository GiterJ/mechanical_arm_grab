#!/usr/bin/env python
# coding: utf-8

import matplotlib
import matplotlib.pyplot as plt
import torch
import cv2
import yaml
from torchvision import transforms
import numpy as np
import open3d as o3d
import warnings

from utils.datasets import letterbox
from utils.general import non_max_suppression_mask_conf

from detectron2.modeling.poolers import ROIPooler
from detectron2.structures import Boxes
from detectron2.utils.memory import retry_if_cuda_oom
from detectron2.layers import paste_masks_in_image


def position_in_camera(rgb, depth, camera_matrix=[[525,0,319],[0,525,239.5],[0,0,1]], target="bottle"):
    """
    :param rgb: opencv image, rgb picture
    :param depth: opencv image, depth picture
    :param target: string, object to grasp
    :param camera_matrix: 2D list, inner parameter of the camera
    :return: cv2 image & list, cv2 image has information on it, list is position of target
    """
    
    # Judge if the input is empty
    if rgb is None or depth is None :
        print("Image is None!")
        return None, None
    
    # load the model file
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    with open('/home/ws_moveit/src/yolov7_mask/data/hyp.scratch.mask.yaml') as f:
        hyp = yaml.load(f, Loader=yaml.FullLoader)
    weigths = torch.load('/home/ws_moveit/src/yolov7_mask/yolov7-mask.pt')
    model = weigths['model']
    # model = model.half().to(device)
    model = model.to(device)
    model = model.float()
    _ = model.eval()

    # process the input image
    image = rgb
    image, ratio, (dw, dh) = letterbox(image, 640, stride=64, auto=True)
    image_ = image.copy()
    image = transforms.ToTensor()(image)
    image = torch.tensor(np.array([image.numpy()]))
    image = image.to(device)
    output = model(image)
    inf_out, train_out, attn, mask_iou, bases, sem_output = output['test'], output['bbox_and_cls'], output['attn'], output['mask_iou'], output['bases'], output['sem']
    bases = torch.cat([bases, sem_output], dim=1)
    nb, _, height, width = image.shape
    names = model.names
    pooler_scale = model.pooler_scale
    pooler = ROIPooler(output_size=hyp['mask_resolution'], scales=(pooler_scale,), sampling_ratio=1, pooler_type='ROIAlignV2', canonical_level=2)
    output, output_mask, output_mask_score, output_ac, output_ab = non_max_suppression_mask_conf(inf_out, attn, bases, pooler, hyp, conf_thres=0.25, iou_thres=0.65, merge=False, mask_iou=None)
    pred, pred_masks = output[0], output_mask[0]
    base = bases[0]

    # Judge if pred is None, aiming at avoid program error but don't figure out why pred is None
    if pred is None:
        print("Not Found {}".format(target))
        return False, rgb, [0,0,0]

    
    bboxes = Boxes(pred[:, :4])
    original_pred_masks = pred_masks.view(-1, hyp['mask_resolution'], hyp['mask_resolution'])
    pred_masks = retry_if_cuda_oom(paste_masks_in_image)( original_pred_masks, bboxes, (height, width), threshold=0.5)
    pred_masks_np = pred_masks.detach().cpu().numpy()
    pred_cls = pred[:, 5].detach().cpu().numpy()
    pred_conf = pred[:, 4].detach().cpu().numpy()
    nimg = image[0].permute(1, 2, 0) * 255
    nimg = nimg.cpu().numpy().astype(np.uint8)
    nimg = cv2.cvtColor(nimg, cv2.COLOR_RGB2BGR)
    nbboxes = bboxes.tensor.detach().cpu().numpy().astype(int) 
    pnimg = nimg.copy()


    pos = [0, 0, 0] #position of the object
    # find the target and calculate its position
    for one_mask, bbox, cls, conf in zip(pred_masks_np, nbboxes, pred_cls, pred_conf):
        flag = 1
        if conf < 0.25: #confidence < 0.25, pass
            continue

        if names[int(cls)]==target:
            flag = 0        #if found the target
            point_rgb = 0   #number of rgb point
            point_depth = 0 #number of depth point
            height = one_mask.shape[0]
            width = one_mask.shape[1]

            # loop
            for i in range(height):
                for j in range(width):
                    if one_mask[i][j]:
                        pos[0] = pos[0] + j
                        pos[1] = pos[1] + i
                        point_rgb = point_rgb + 1
                        k = int(i-dh)
                        t = int(j-dw)
                        if depth[k][t] == 0:
                            continue
                        pos[2] = pos[2] + depth[k][t]
                        point_depth = point_depth + 1
            
            #calculate the averag and eliminate padding and zoom
            pos[0] = pos[0]/point_rgb
            pos[1] = pos[1]/point_rgb
            pos_orix = pos[0]
            pos_oriy = pos[1]
            pos[0] = ((pos[0]-dw)/(width-2*dw))*rgb.shape[1]
            pos[1] = ((pos[1]-dh)/(height-2*dh))*rgb.shape[0]
            pos[2] = pos[2]/point_depth

            # visualization
            color = [np.random.randint(255), np.random.randint(255), np.random.randint(255)]
            pnimg[one_mask] = pnimg[one_mask] * 0.5 + np.array(color, dtype=np.uint8) * 0.5
            pnimg = cv2.rectangle(pnimg, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
            label = '%s %.3f' % (names[int(cls)], conf)
            t_size = cv2.getTextSize(label, 0, fontScale=0.5, thickness=1)[0]
            c2 = bbox[0] + t_size[0], bbox[1] - t_size[1] - 3
            pnimg = cv2.rectangle(pnimg, (bbox[0], bbox[1]), c2, color, -1, cv2.LINE_AA)
            pnimg = cv2.putText(pnimg, label, (bbox[0], bbox[1] - 2), 0, 0.5, [255, 255, 255], thickness=1, lineType=cv2.LINE_AA) 
            cv2.circle(pnimg, (int(pos_orix), int(pos_oriy)), 5, (0,0,0), -1) 
            break
    
    # Return after considering camera matrix
    if flag:
        print("Not Found {}".format(target))
        return False, rgb, [0,0,0]
    else:
        camera_matrix_inv = np.linalg.inv(camera_matrix)
        P_uv = np.array([pos[0], pos[1], 1])
        P_xyz = pos[2] * np.dot(camera_matrix_inv, P_uv)/1000
        P_xyz[2] = P_xyz[2]
        print("Found <{}>, position in camera frame: {}".format(target, list(P_xyz))) 
        return True, pnimg, list(P_xyz)


def param2matrix(fx, fy, cx, cy):
    """
    Transform camera parameter to matrix
    :param fx,fy,cx,cy: camera parameter
    :return: camera parameter matrix
    """
    return [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]


if __name__ == "__main__":
    warnings.filterwarnings("ignore")
    # opencv read image
    depth = cv2.imread("/home/jlinux/YOLO/0697490ca908ef2cc99826951001f7b.png", -1)
    image = cv2.imread("/home/jlinux/YOLO/e45599f06e5aea7a2290602794aaac0.jpg")

    # set camera matrix
    camera_matrix = param2matrix(378.998657, 378.639862, 321.935120, 240.766663)
    print("camera matrix:",camera_matrix)

    # get the position
    img, position = position_in_camera(image, depth, camera_matrix, target="bottle")
    print("Position in camera frame:",position)

    # show the position of the target
    cv2.imshow("img", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
