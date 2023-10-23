#!/usr/bin/python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from sawyer_pykdl import sawyer_kinematics


def main():
    rospy.init_node('sawyer_kinematics')
    print '*** Sawyer PyKDL Kinematics ***\n'
    kin = sawyer_kinematics('right')

    print '\n*** Sawyer Description ***\n'
    kin.print_robot_description()
    print '\n*** Sawyer KDL Chain ***\n'
    kin.print_kdl_chain()
    # FK Position
    print '\n*** Saywer Position FK ***\n'
    print kin.forward_position_kinematics()
    # FK Velocity
    # print '\n*** Sawyer Velocity FK ***\n'
    # kin.forward_velocity_kinematics()
    # IK
    print '\n*** Sawyer Position IK ***\n'
    pos = [0.582583, -0.180819, 0.216003]
    rot = [0.03085, 0.9945, 0.0561, 0.0829]
    print kin.inverse_kinematics(pos)  # position, don't care orientation
    print '\n*** Sawyer Pose IK ***\n'
    print kin.inverse_kinematics(pos, rot)  # position & orientation
    # Jacobian
    print '\n*** Sawyer Jacobian ***\n'
    print kin.jacobian()
    # Jacobian Transpose
    print '\n*** Sawyer Jacobian Tranpose***\n'
    print kin.jacobian_transpose()
    # Jacobian Pseudo-Inverse (Moore-Penrose)
    print '\n*** Sawyer Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
    print kin.jacobian_pseudo_inverse()
    # Joint space mass matrix
    print '\n*** Sawyer Joint Inertia ***\n'
    print kin.inertia()
    # Cartesian space mass matrix
    print '\n*** Sawyer Cartesian Inertia ***\n'
    print kin.cart_inertia()
    # Joint space coriolis matrix
    print '\n*** Sawyer Joint Coriolis ***\n'
    print kin.coriolis()
    # Cartesian space coriolis matrix
    # print '\n*** Sawyer Cartesian Coriolis ***\n'
    # print kin.cart_coriolis()
    # Joint space gravity matrix
    print '\n*** Sawyer Joint Gravity ***\n'
    print kin.gravity()
    # Cartesian space gravity matrix
    # print '\n*** Baxter Cartesian Gravity ***\n'
    # print kin.cart_gravity()

if __name__ == "__main__":
    main()
