# -*- coding: utf-8 -*-
"""
@author: Marcos F. Caetano (mfcaetano@unb.br) 03/11/2020

@description: PyDash Project

An implementation example of a FIXED R2A Algorithm.

the quality list is obtained with the parameter of handle_xml_response() method and the choice
is made inside of handle_segment_size_request(), before sending the message down.

In this algorithm the quality choice is always the same.
"""

from player.parser import *
from r2a.ir2a import IR2A

from collections import namedtuple
import numpy as np #usaremos para random
import time
from math import floor

class R2AQLearning(IR2A):

	##############################################################
	##### Realizacao das funcoes abstratas herdadas de IR2A  #####
	##############################################################
    def __init__(self, id):
        IR2A.__init__(self, id)
        self.parsed_mpd = ''
        self.qi = []
	# number of quality levels 
	# value obtained in handle_xml_response()
        self.N = 0 
        self.request_time = 0

    # buffers
        self.buffer_max = 0
        self.buffer_anterior = 1
        self.buffer_atual = 0

    #oscilação
        self.qi_atual = 0
        self.qi_anterior = 0
        self.qi_antepenultimo = 0
        self.oscillation = 0
        self.max_length = 30
        self.length = 0
        self.depht = 0

	###################### State definiion #######################
	# first argument: current quality
	# second argument: current bandwidth
        self.state = [0,0]
        self.low_bandwidth = 0
        self.medium_bandwidth = 0
        self.high_bandwidth = 0
		
	
	############# Constants used to define the reward ############
	# Wieghts C1-C4 - the values ​​used were obtained in the article
        self.C1 = 2
        self.C2 = 1
        self.C3 = 4
        self.C4 = 3

    def handle_xml_request(self, msg):
        self.request_time = time.time()
        self.send_down(msg)

    def handle_xml_response(self, msg):
        # getting qi list
        self.parsed_mpd = parse_mpd(msg.get_payload())
        self.qi = self.parsed_mpd.get_qi()
        self.N = len(self.qi)
		
        self.low_bandwidth = self.qi[0]
        self.medium_bandwidth = self.qi[floor(self.N/2) - 1]
        self.high_bandwidth = self.qi[self.N - 1]
		
		# so consigo criar a tabela quando souber quantas qualidades tem
		# e as faixas de largura de banda
        self.create_q_table()
		
        t = time.time() - self.request_time #diferenca de tempo
        self.bandwidth = msg.get_bit_length()/t #bits/s
        print('Bandwidth: ', self.bandwidth)
		# mantem a qualidade e atualiza o bandwidth
        self.state[1] = self.bandwidth
        print('State: ', self.state)
		
        self.send_up(msg)

    def handle_segment_size_request(self, msg):
	
        self.request_time = time.time()

        # time to define the segment quality choose to make the request
        self.state[0] = self.qi[19] #Alterar depois
        msg.add_quality_id(self.qi[19]) #Aqui que colocamos a qualidade
        self.send_down(msg)

    def handle_segment_size_response(self, msg):
        t = time.time() - self.request_time #diferenca de tempo
        self.bandwidth = msg.get_bit_length()/t #bits/s
        print('Bandwidth: ', self.bandwidth)
		
		#calcular recomensa
        #rodar o q-learning
		
		# mantem a qualidade (que ainda sera alterada)
		# e atualiza o bandwidth



        if len(self.whiteboard.get_playback_buffer_size())!=0:
       	    self.buffer_anterior = self.buffer_atual
            self.buffer_atual = self.whiteboard.get_playback_buffer_size()[-1][1]

        if len(self.whiteboard.get_playback_qi())!=0:
            self.qi_antepenultimo = self.qi_anterior
            self.qi_anterior = self.qi_atual
            self.qi_atual = self.whiteboard.get_playback_qi()[-1][1]

        self.state[1] = self.bandwidth
        print('State: ', self.state)
        self.send_up(msg)

    def initialize(self):
    	self.buffer_max = self.whiteboard.get_max_buffer_size()
    	pass

    def finalization(self):
        pass
		
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
	
	# Knowing how many states exist, initialize the Q-table with 0
    def create_q_table(self):
        self.q_table = np.zeros((5, self.N)) #5 faixas de bandwidth
	
	##############################################################
	#########              Reward Functions              #########
	##############################################################
		
	# R_quality
    def reward_quality(self):
        r_quality = (self.qi_atual-1)/(self.N-1)*2 - 1 #formula do artigo
        return r_quality

	# R_oscillation
    def reward_oscillation(self):
		# vamos precisar de um vetor de qualidades antigas para
		# definir a profundidade e largura da oscilacao - quantos
		# valores precisamos, sera?
		
		# arquivo player.py parece ter informacoes do buffer:
		# self.max_buffer_size = int(config_parser.get_parameter('max_buffer_size'))
		# self.playback_buffer_size = OutVector() seria o quao cheio esta o buffer?
		#		nao sei o que seria esse OutVector

        if ((self.qi_antepenultimo >= self.qi_anterior) & (self.qi_atual >= self.qi_anterior)) | ((self.qi_antepenultimo <= self.qi_anterior) & (self.qi_atual <= self.qi_anterior)):
            self.oscillation = 0
            self.length += 1
        else:
            self.oscillation = 1
            self.depth = self.qi_atual - self.qi_anterior

        if self.oscillation == 0:
        	r_oscillation = 0
        else:
        	if self.length >= self.max_length:
        		r_oscillation = 0
        	else:
        	    r_oscillation = -1/float(self.length)**(2/self.depth) + (float(self.lenght)-1)/((self.max_length-1)*self.max_length**(2/depth))

        	self.length = 0

        return r_oscillation
		
	# R_bufferfilling
    def reward_bufferfilling(self):
    	if self.buffer_atual <= 0.1*self.buffer_max:
            r_bufferfilling = -1
    	else:
            r_bufferfilling = (2*buffer_atual)/0.9*buffer_max - 1.1/0.9   
        
    	return r_bufferfilling
	
	# R_bufferchange
    def reward_bufferchange(self):
        if self.buffer_atual <= self.buffer_anterior:
            r_bufferchange = (self.buffer_atual - self.buffer_anterior)/self.buffer_anterior
        else:
            r_bufferchange = (self.buffer_atual - self.buffer_anterior)/(self.buffer_atual - self.buffer_anterior/2) 

        return r_bufferchange
		
	# R - total reward
    def total_reward(self):
        r_quality = self.reward_quality()
        r_oscillation = self.reward_oscillation()
        r_bufferfilling = self.reward_bufferfilling()
        r_bufferchange = self.reward_bufferchange()
		
        reward = self.C1*r_quality + self.C2*r_oscillation + self.C3*r_bufferfilling + self.C4*r_bufferchange
        return reward
	
	##############################################################
	#########             Exploration Policy             #########
	##############################################################
	
	# Como decidir entre exploration e exploitation
	# Pelos resultados do artigo, parece melhor usar o VDBE-Softmax
	# maaaas, eu usaria o e-greedy pq é bem mais simples de 
	# implementar, e o prof disse q n precisa ser tao fiel ao artigo
	
	# Achei possivel, mas complicadinho implementar o Softmax ou o
	# VDBE-Softmax: 
	# http://www.tokic.com/www/tokicm/publikationen/papers/KI2011.pdf
	# Judeu implementou Softmax, podemos ver pelo dele tmb
	
	####################### Using ε-greedy #######################
    def e_greedy(self):
		# Exploration rate (ε)
        epsilon = 0.05
        random_number = np.random.random()
        if random_number < epsilon:
            self.exploration()
        else:
            self.exploitation()
	
	######################## Exploration #########################
	
    def exploration(self):
	############ Constants used to update the Q-value ############
		# The values ​​used were obtained in the article
		# learning rate (α)
        alfa = 0.3
		# discount factor (γ)
        gama = 0.95
	
	# seleciona aleatoriamente o próximo estado e atualiza a 
	# tabela Q com o resultado de Q(s, a) = Q(s, a) + α [r + γ
	# max_b(s',b) - Q(s,a)] - Bellman Equation
	
	######################## Exploitation ########################
	
    def exploration(self):
        print('em exploration')
	# so usa a tabela Q
