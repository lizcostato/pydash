# -*- coding: utf-8 -*-
"""
@author: Alexandre Abrahami Pinto da Cunha - 18/0041169
@author: Fernanda Vaz Borges Carneiro - 18/0052705
@author: Liz Carolina Jaber Costato - 18/0022261

@description: PyDash Project

An implementation example of a FIXED R2A Algorithm.

the quality list is obtained with the parameter of handle_xml_response() method and the choice
is made inside of handle_segment_size_request(), before sending the message down.

In this algorithm the quality choice is made using the q-learning algorithm.
"""

from player.parser import *
from player.player import *
from r2a.ir2a import IR2A

from collections import namedtuple
import numpy as np #usaremos para random
import time
import random
import pandas
from math import floor

class R2AQLearning(IR2A):

	##############################################################
	### Realization of abstract functions inherited from IR2A ####
	##############################################################
    def __init__(self, id):
        IR2A.__init__(self, id)
        self.parsed_mpd = ''
        self.qi = []
	
        self.N = 0  # number of quality levels
        self.request_time = 0
        self.qi_request = 0

    # buffers
        self.buffer_max = self.whiteboard.get_max_buffer_size()
        self.buffer_anterior = 1
        self.buffer_atual = 0

    # oscilation
        self.qi_atual = 0
        self.qi_anterior = 0
        self.oscillation = 0
        self.increase = True
        self.decrease = False
        self.max_length = 30
        self.length = 1
        self.depht = 0

	###################### State definiion #######################
	# first argument: current quality
	# second argument: current bandwidth
        self.state = [0,0]
        self.last_state = [0,0]
        self.low_bandwidth = 0
        self.low_medium_bandwidth = 0
        self.high_medium_bandwidth = 0
        self.high_bandwidth = 0
        self.SL = 0
        self.L = 1
        self.M = 2
        self.H = 3
        self.SH = 4
        self.qi_vector = []
	
	############# Constants used to define the reward ############
	# Wieghts C1-C4 - the values ​​used were obtained in the article
        self.C1 = 2
        self.C2 = 1
        self.C3 = 4
        self.C4 = 3
		
	############ Constants used to update the Q-value ############
	# The values ​​used were obtained in the article
	# learning rate (α)
        self.alfa = 0.3
	# discount factor (γ)
        self.gama = 0.95

    # Vectors used to create images
        self.vector_reward = []
        self.vector_time_reward = []

    # Getting Q_table from qtable.txt file
        self.q_table = np.loadtxt('q_table_interval_25_profile_LLLMMMHMMMH_softmax.txt')


    def handle_xml_request(self, msg):
        self.request_time = time.time()
        self.send_down(msg)

    def handle_xml_response(self, msg):
        # getting qi list
        self.parsed_mpd = parse_mpd(msg.get_payload())
        self.qi = self.parsed_mpd.get_qi()
        self.N = len(self.qi)
        for i in range(self.N):
            self.qi_vector.append(i)
		
        self.low_bandwidth = self.qi[0]
        self.low_medium_bandwidth = self.qi[floor(self.N/3) - 1]
        self.high_medium_bandwidth = self.qi[floor(2*self.N/3) - 1]
        self.high_bandwidth = self.qi[self.N - 1]
		
        # self.create_q_table()
		
        t = time.time() - self.request_time 
        self.bandwidth = msg.get_bit_length()/t #bits/s
       
        # define which of the 5 ranges our current band is on
        self.last_state = self.state
        if self.bandwidth < self.low_bandwidth:
            self.state[1] = self.SL
        elif self.bandwidth < self.low_medium_bandwidth:
            self.state[1] = self.L	
        elif self.bandwidth < self.high_medium_bandwidth:
            self.state[1] = self.M
        elif self.bandwidth < self.high_bandwidth:
            self.state[1] = self.H
        elif self.bandwidth > self.high_bandwidth:
            self.state[1] = self.SH
		
        self.send_up(msg)

    def handle_segment_size_request(self, msg):
        self.request_time = time.time()

        self.update_q_table()
        msg.add_quality_id(self.qi[self.qi_request])
        
        self.last_state = self.state
        self.state[0] = self.qi_atual
        
        self.send_down(msg)

    def handle_segment_size_response(self, msg):
        t = time.time() - self.request_time #diferenca de tempo
        self.bandwidth = msg.get_bit_length()/t #bits/s
		
        # define which of the 5 ranges our current band is on
        self.last_state = self.state
        if self.bandwidth < self.low_bandwidth:
            self.state[1] = self.SL
        elif self.bandwidth < self.low_medium_bandwidth:
            self.state[1] = self.L	
        elif self.bandwidth < self.high_medium_bandwidth:
            self.state[1] = self.M
        elif self.bandwidth < self.high_bandwidth:
            self.state[1] = self.H
        elif self.bandwidth > self.high_bandwidth:
            self.state[1] = self.SH

        # getting buffer information from the whiteboard
        if len(self.whiteboard.get_playback_buffer_size())!=0:
       	    self.buffer_anterior = self.buffer_atual
            self.buffer_atual = self.whiteboard.get_playback_buffer_size()[-1][1]

        # getting quality information from the whiteboard
        if len(self.whiteboard.get_playback_qi())!=0:
            self.qi_anterior = self.qi_atual
            self.qi_atual = self.whiteboard.get_playback_qi()[-1][1]

        # calling the softmax function to chose the next quality to be requested
        self.table_line = self.q_table[self.N*self.state[1] + self.state[0], :]
        self.qi_request = self.probability_function(self.softmax(self.table_line), self.qi_vector)

        self.send_up(msg)

    def initialize(self):
    	pass

    def finalization(self):
        np.savetxt("q_table_interval_25_profile_LLLMMMHMMMH_softmax.txt", self.q_table)
        self.graphic(self.vector_reward, self.vector_time_reward, 'reward', 'Reward', 'reward')
        
        

		
	##############################################################
	#########                   Q-table                  #########
	##############################################################
	#						Actions (choose qi[x])
	#	States		 | qi[0] qi[1] qi[2] qi[3] ... qi[N-1] qi[N]
	# qi[0], BW = SL |
	# qi[1], BW = SL |
	# qi[2], BW = SL |
	# qi[3], BW = SL |
	# ...            |
	# qi[N], BW = SL |              Q-values
	# qi[0], BW = L  | 
	# qi[1], BW = L  |
	# ...            | 	
	# qi[N], BW = L  | 
	# qi[0], BW = M  |
	# qi[1], BW = M  |
	# qi[2], BW = M  |
	# ...            |
	# qi[N], BW = SH |
	

	# Initialize the Q-table with 0, only runs the first time the algorithm is executed
    def create_q_table(self):
        self.q_table = np.zeros((5*self.N, self.N))
	

	##############################################################
	#########              Reward Functions              #########
    #########          (were obtained in the article)    #########
	##############################################################
		
	# R_quality
    def reward_quality(self):
        r_quality = (((self.qi_atual-1)/(self.N-1))*2) - 1 
        return r_quality

	# R_oscillation
    def reward_oscillation(self):
        if self.qi_atual > self.qi_anterior:
            if self.increase == True:
                self.oscillation = 0
                self.length += 1
            else:
                self.increase = True
                self.decrease = False
                self.oscillation = 1
                self.depth = abs(self.qi_atual - self.qi_anterior)
        elif self.qi_atual < self.qi_anterior:
            if self.decrease == True:
                self.oscillation = 0
                self.length += 1
            else:
                self.increase = False
                self.decrease = True
                self.oscillation = 1
                self.depth = abs(self.qi_atual - self.qi_anterior)
        else:
            self.oscillation = 0
            self.length += 1

        if self.oscillation == 0:
            r_oscillation = 0
        else:
            if self.length >= self.max_length:
                r_oscillation = 0
            else:
                r_oscillation = -1/float(self.length)**(2/self.depth) + (float(self.length)-1)/((self.max_length-1)*self.max_length**(2/self.depth))
            self.length = 1

        return r_oscillation
		
	# R_bufferfilling
    def reward_bufferfilling(self):
    	if self.buffer_atual <= 0.1*self.buffer_max:
            r_bufferfilling = -1
    	else:
            r_bufferfilling = ((2*self.buffer_atual)/(0.9*self.buffer_max)) - (1.1/0.9) 
    	return r_bufferfilling
	
	# R_bufferchange
    def reward_bufferchange(self):
        if self.buffer_atual <= self.buffer_anterior:
            if self.buffer_anterior != 0:
                r_bufferchange = (self.buffer_atual - self.buffer_anterior)/self.buffer_anterior
            else:
                r_bufferchange = -1
        else:
            r_bufferchange = (self.buffer_atual - self.buffer_anterior)/(self.buffer_atual - (self.buffer_anterior/2)) 
        return r_bufferchange
		
	# R - total reward
    def total_reward(self):
        r_quality = self.reward_quality()
        r_oscillation = self.reward_oscillation()
        r_bufferfilling = self.reward_bufferfilling()
        r_bufferchange = self.reward_bufferchange()
        reward = self.C1*r_quality + self.C2*r_oscillation + self.C3*r_bufferfilling + self.C4*r_bufferchange

        #vectors used to create the graph
        self.vector_reward.append(reward)
        self.vector_time_reward.append(time.time())
        return reward
	
	##############################################################
	#########             Exploration Policy             #########
	##############################################################

    ##################### Softmax ############################# 
    def softmax(self, x):
        e_x = np.exp(x)
        return e_x / e_x.sum()

    def probability_function(self, probability_vector, quality_vector):
        return random.choices(quality_vector, weights=probability_vector, k=1)[0]

    def update_q_table(self):
        indice1 = self.N*self.last_state[1] + self.last_state[0] # table's line = N*BW + qi
        indice2 = self.qi_request # table's colunn = quality
        indice3 = self.N*self.state[1] + self.state[0] # table's line = N*BW + qi

        #Bellman's equation
        self.q_table[indice1][indice2] = self.q_table[indice1][indice2]+self.alfa*(self.total_reward()+self.gama*np.max(self.q_table[indice3,:]) - self.q_table[indice1][indice2]) 

    ###################### Graph generation #########################
    def graphic(self, vector_inf, vector_time, file_name, title, y_axis, x_axis='execution time(s)'):

        plt.plot(vector_time, vector_inf, label=file_name)
        plt.xlabel(x_axis)
        plt.ylabel(y_axis)
        plt.title(title)

        plt.savefig(f'./results/{file_name}.png')
        plt.clf()
        plt.cla()
        plt.close()