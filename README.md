# mechanical_arm_grab

*物体识别与抓取，硬件基于Intel realsense D435，Diana7 机械臂和 Jodell RG52夹爪，识别基于yolov7-mask，抓取位姿基于GPD生成，轨迹规划基于moveit* 

https://github.com/GiterJ/mechanical_arm_grab/assets/113225690/ff695767-74cf-4c9e-94a3-558c4b7532e3


## Attention

- 需要下载 `yolov7-mask.pt` 和 `traced_model.pt` 模型

- 需要用 `gpd` 源码中的 `models` 目录替换此仓库中的 `/gpd/models` 目录 (`caffe&lenet&openvino`)，然后在 gpd 目录下进行 make

- `diana_description/urdf` 目录下需要放入机器臂 urdf 文件


## Docker

- [Docker Image](https://hub.docker.com/r/jjocker/mechanical_arm_grab)