#! /usr/bin/env python

# /***************************************************************************

# 
# @package: franka_interface
# @metapackage: franka_ros_interface
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
# 

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2020, Saif Sidhik
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/


"""
 @info: 
   commands robot to move to neutral pose

"""

import rospy
from franka_interface import ArmInterface, GripperInterface

if __name__ == '__main__':
    rospy.init_node("pickplace_node")

    arm = ArmInterface()
    gripper = GripperInterface()

    pick_q = {'panda_joint1': -0.439750975483344, 'panda_joint2': 0.8899523907596788, 'panda_joint3': 0.024437583407033177, 'panda_joint4': -1.537275188981441, 'panda_joint5': -0.04936081370520662, 'panda_joint6': 2.4703837054510696, 'panda_joint7': 0.4288298841891062}
    pick_q_approach = {'panda_joint1': -0.4221961033310468, 'panda_joint2': 0.8192085726977326, 'panda_joint3': 0.0110135298822931, 'panda_joint4': -1.5119577281541396, 'panda_joint5': -0.05116776515973339, 'panda_joint6': 2.3594606653091303, 'panda_joint7': 0.4372254975173934}
    place_q = {'panda_joint1': 0.581441363945342, 'panda_joint2': 1.0090749857512291, 'panda_joint3': -0.10068985400763025, 'panda_joint4': -1.3040675308495238, 'panda_joint5': -0.05447560265966894, 'panda_joint6': 2.2908197953447678, 'panda_joint7': 1.28984750977662}
    place_q_approach = {'panda_joint1': 0.5671383125889078, 'panda_joint2': 0.883704280167317, 'panda_joint3': -0.0943985838601151, 'panda_joint4': -1.346540315661514, 'panda_joint5': -0.0529579417109989, 'panda_joint6': 2.288061349254975, 'panda_joint7': 1.2911698020036582}

    
    gripper.open()
    arm.move_to_neutral()

    arm.move_to_joint_positions(pick_q_approach)
    arm.move_to_joint_positions(pick_q)
    gripper.close()

    arm.move_to_neutral()

    arm.move_to_joint_positions(place_q_approach)
    arm.move_to_joint_positions(place_q)
    gripper.open()
    arm.move_to_joint_positions(place_q_approach)
    arm.move_to_neutral()

