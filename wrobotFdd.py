# 1、请在终端执行：
# docker run -it \
#   -v .:/data \
#   -p 7000:7000 \
#   --device=/dev/dri \
#   --group-add video \
#   --volume=/tmp/.X11-unix:/tmp/.X11-unix \
#   --env="DISPLAY=$DISPLAY" \
#   --env="QT_X11_NO_MITSHM=1" \
#   --name=pinU20 \
#   ubuntu20_pino_cro:v0 /bin/bash
# 2、请执行 
# cd /data
# 3、请执行
# python3 wrobotFdd.py

import os
import sys
import numpy as np
import itertools
import math
sys.path.append("/opt/openrobots/lib/python3.8/site-packages")
from pinocchio import visualize
import pinocchio
import example_robot_data
import crocoddyl
from pinocchio.robot_wrapper import RobotWrapper
import time

current_directory = os.getcwd()
print("上层路径：", current_directory)

# change path ??
modelPath = current_directory + '/resources/robots/wrobot/'
URDF_FILENAME = "urdf/wrobot_Fdd.urdf"
print("模型路径：", modelPath + URDF_FILENAME)

# Load the full model
rrobot = RobotWrapper.BuildFromURDF(modelPath + URDF_FILENAME, [modelPath], pinocchio.JointModelFreeFlyer())  # Load URDF file
rmodel = rrobot.model

rightFoot = 'right_ankle_link'
leftFoot = 'left_ankle_link'

display = crocoddyl.MeshcatDisplay(
    rrobot, frameNames=[rightFoot, leftFoot]
)
q0 = pinocchio.utils.zero(rrobot.model.nq)
display.display([q0])
time.sleep(0.01)
# print(1)

rdata = rmodel.createData()
pinocchio.forwardKinematics(rmodel, rdata, q0)
pinocchio.updateFramePlacements(rmodel, rdata)

rfId = rmodel.getFrameId(rightFoot)
lfId = rmodel.getFrameId(leftFoot)

rfFootPos0 = rdata.oMf[rfId].translation
lfFootPos0 = rdata.oMf[lfId].translation

comRef = pinocchio.centerOfMass(rmodel, rdata, q0)


# print(1)

# initialAngle = np.array([0.3, 0.1, 0.3, -0.5, -0.2, 0.0,
#                          0.3, 0.1, 0.3, -0.5, -0.2, 0.0,
#                          0.0, 0.00, 0.0,
#                          0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,
#                          0.0, 0.0, 0.0,
#                          0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
# q0 = pinocchio.utils.zero(rrobot.model.nq)
# q0[6] = 1  # q.w
# q0[2] =0.5848  # z
# q0[ 7:39] = initialAngle
# display.display([q0])

for i in range(rrobot.model.nq-7):
    q0 = pinocchio.utils.zero(rrobot.model.nq)
    q0[6] = 1  # q.w
    q0[2] = 0  # z
    q0[i+7] = 1
    display.display([q0])
    time.sleep(0.01)
    print(1)


for i in range(1000):
    phase = i * 0.005
    sin_pos = np.sin(2 * np.pi * phase)
    sin_pos_l = sin_pos.copy()
    sin_pos_r = sin_pos.copy()

    ref_dof_pos = np.zeros((1,10))
    scale_1 = 0.17
    scale_2 = 2 * scale_1
    # left foot stance phase set to default joint pos
    if sin_pos_l > 0 :
        sin_pos_l = sin_pos_l * 0
    if abs(sin_pos_l) < 0.1:
        sin_pos_l = sin_pos_l * 0
    ref_dof_pos[:, 2] = sin_pos_l * scale_1 - 0.45
    ref_dof_pos[:, 3] = -sin_pos_l * scale_2 + 1.1
    ref_dof_pos[:, 4] = sin_pos_l * scale_1 - 0.6
    # right foot stance phase set to default joint pos
    if sin_pos_r < 0:
        sin_pos_r = sin_pos_r * 0
    if abs(sin_pos_r) < 0.1:
        sin_pos_r = sin_pos_r * 0
    ref_dof_pos[:, 7] = -sin_pos_r * scale_1 - 0.45
    ref_dof_pos[:, 8] = sin_pos_r * scale_2 + 1.1
    ref_dof_pos[:, 9] = -sin_pos_r * scale_1 - 0.6
    # Double support phase
    # ref_dof_pos[np.abs(sin_pos) < 0.1] = 0

    q0 = pinocchio.utils.zero(rrobot.model.nq)
    q0[6] = 1  # q.w
    q0[2] = 0  # z
    q0[7:rrobot.model.nq] = ref_dof_pos
    display.display([q0])
    time.sleep(0.01)
    print(1)





for i in range(rrobot.model.nq-7):
    q0 = pinocchio.utils.zero(rrobot.model.nq)
    q0[6] = 1  # q.w
    q0[2] = 0  # z
    q0[i+7] = 1
    display.display([q0])
    time.sleep(0.01)
    print(1)
