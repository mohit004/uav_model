#!/usr/bin/env python

###########################################################
### @file   dnn.py
### @author Efe Camci, Mohit Mehndiratta (Nanyang Technological University)
### @date   Feb 2019
###
### @copyright
### Copyright (C) 2019.
###########################################################

# -*- coding: utf-8 -*-
from nmpc_weight_tuning_by_rl.srv import *
from std_msgs.msg import Float64MultiArray
import rospy
import torch
import torch.nn as nn
import torch.nn.functional as F

import numpy as np
import pandas as pd
import random
import sys
import os
import math

class Parameters:
    history_length = 40

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()

	architecture = 0

	if (architecture == 0):
	    # Architecture 0
            self.fc00 = nn.Linear(3*Parameters.history_length,16)
            self.fc01 = nn.Linear(16,16)

            self.fc10 = nn.Linear(3*Parameters.history_length,16)
            self.fc11 = nn.Linear(16,16)

            self.fc20 = nn.Linear(3*Parameters.history_length,16)
            self.fc21 = nn.Linear(16,16)

            self.fc30 = nn.Linear(3*Parameters.history_length,16)
            self.fc31 = nn.Linear(16,16)

            self.fc40 = nn.Linear(4*Parameters.history_length,32)
            self.fc41 = nn.Linear(32,32)

            self.fc50 = nn.Linear(64,32)
            self.fc51 = nn.Linear(32,32)

            self.fc60 = nn.Linear(64,32)
            self.fc61 = nn.Linear(32,12)

	elif (architecture == 1):

            # Architecture 1
            self.fc00 = nn.Linear(3*Parameters.history_length,128)
            self.fc01 = nn.Linear(128,16)

            self.fc10 = nn.Linear(3*Parameters.history_length,128)
            self.fc11 = nn.Linear(128,16)

            self.fc20 = nn.Linear(3*Parameters.history_length,128)
            self.fc21 = nn.Linear(128,16)

            self.fc30 = nn.Linear(3*Parameters.history_length,128)
            self.fc31 = nn.Linear(128,16)

            self.fc40 = nn.Linear(4*Parameters.history_length,256)
            self.fc41 = nn.Linear(256,32)

            self.fc50 = nn.Linear(64,32)
            self.fc51 = nn.Linear(32,32)

            self.fc60 = nn.Linear(64,32)
            self.fc61 = nn.Linear(32,12)

    def forward(self, position, velocity, angle, angular_rate, commands):

        position = F.relu(self.fc01(F.relu(self.fc00(position))))
        velocity = F.relu(self.fc11(F.relu(self.fc10(velocity))))
        angle = F.relu(self.fc21(F.relu(self.fc20(angle))))
        angular_rate = F.relu(self.fc31(F.relu(self.fc30(angular_rate))))
        commands = F.relu(self.fc41(F.relu(self.fc40(commands))))

        position = torch.cat((position,velocity),1)
        position = torch.cat((position,angle),1)
        position = torch.cat((position,angular_rate),1)

        position = F.relu(self.fc51(F.relu(self.fc50(position))))
        position = torch.cat((position,commands),1)

        position = self.fc61(F.relu(self.fc60(position)))

        return position

class Network:
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))
    network_file = path_to_this_file[0:(len(path_to_this_file)-3)] + 'data/model/network/dnn'
    net = Net()
    if(os.path.isfile(network_file)):
        net.load_state_dict(torch.load(network_file))

def dnn_fun(req):
    data_py = []
    data_py = req.states_and_commands
    data = torch.tensor(data_py)

    position = torch.zeros(1,3*Parameters.history_length)
    velocity = torch.zeros(1,3*Parameters.history_length)
    angle = torch.zeros(1,3*Parameters.history_length)
    angular_rate = torch.zeros(1,3*Parameters.history_length)
    commands = torch.zeros(1,4*Parameters.history_length)

    max_position_xy = 1.6931
    max_position_z = 1.7178
    max_velocity_xy = 2.6095
    max_velocity_z = 2.3099
    max_angle_xy = 0.5738
    max_angle_z = 0.2598
    max_angular_rate_xy = 4.5316
    max_angular_rate_z = 1.0995
    max_commands_1 = 0.6109
    max_commands_2 = 0.6109
    max_commands_3 = 0.1410
    max_commands_4 = 13.5084
    for i in range(Parameters.history_length):
        position[0,i*3] = data[i*16]/max_position_xy
        position[0,i*3+1] = data[i*16+1]/max_position_xy
        position[0,i*3+2] = data[i*16+2]/max_position_z

        velocity[0,i*3] = data[i*16+3]/max_velocity_xy
        velocity[0,i*3+1] = data[i*16+4]/max_velocity_xy
        velocity[0,i*3+2] = data[i*16+5]/max_velocity_z

        angle[0,i*3] = data[i*16+6]/max_angle_xy
        angle[0,i*3+1] = data[i*16+7]/max_angle_xy
        angle[0,i*3+2] = data[i*16+8]/max_angle_z

        angular_rate[0,i*3] = data[i*16+9]/max_angular_rate_xy
        angular_rate[0,i*3+1] = data[i*16+10]/max_angular_rate_xy
        angular_rate[0,i*3+2] = data[i*16+11]/max_angular_rate_z

        commands[0,i*4] = data[i*16+12]/max_commands_1
        commands[0,i*4+1] = data[i*16+13]/max_commands_2
        commands[0,i*4+2] = data[i*16+14]/max_commands_3
        commands[0,i*4+3] = data[i*16+15]/max_commands_4

    # Cuda
#    net.cuda()
#    position, velocity, angle, angular_rate, commands = position.cuda(), velocity.cuda(), angle.cuda(), angular_rate.cuda(), commands.cuda()

    prediction = Network.net(position,velocity,angle,angular_rate,commands)

    prediction[0,0:2] = prediction[0,0:2]*max_position_xy
    prediction[0,2] = prediction[0,2]*max_position_z
    prediction[0,3:5] = prediction[0,3:5]*max_velocity_xy
    prediction[0,5] = prediction[0,5]*max_velocity_z
    prediction[0,6:8] = prediction[0,6:8]*max_angle_xy
    prediction[0,8] = prediction[0,8]*max_angle_z
    prediction[0,9:11] = prediction[0,9:11]*max_angular_rate_xy
    prediction[0,11] = prediction[0,11]*max_angular_rate_z

    predicted_state = []
    predicted_state = prediction[0,:]

#    print(predicted_state)

    return DnnResponse(predicted_state)

def dnn_server():
    rospy.init_node('dnn_server')
    s = rospy.Service('dnn', Dnn, dnn_fun)
    rospy.spin()

if __name__ == "__main__":
    dnn_server()
