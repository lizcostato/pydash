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

class R2AQLearning(IR2A):

	##############################################################
	##### Realizacao das funcoes abstratas herdadas de IR2A  #####
	##############################################################
    def __init__(self, id):
        IR2A.__init__(self, id)
        self.parsed_mpd = ''
        self.qi = []
		#number of quality levels 
		# value obtained in handle_xml_response()
        self.N = 0 

	##############################################################
	#########              State definiion               #########
	##############################################################
	#two parameters: current quality and bandwidth
	#Defini como tupla pq será sempre o mesmo nº de parâmetros: 2
	#Segui https://blog.betrybe.com/tecnologia/tuplas-em-python/
        self.States = namedtuple('States', ['Quality', 'Bandwidth'])
        self.state = self.States(0,0)
	
	############# Constants used to define the reward ############
	# Wieghts C1-C4 - the values ​​used were obtained in the article
        self.C1 = 2
        self.C2 = 1
        self.C3 = 4
        self.C4 = 3

    def handle_xml_request(self, msg):
        self.send_down(msg)

    def handle_xml_response(self, msg):
        # getting qi list
        self.parsed_mpd = parse_mpd(msg.get_payload())
        self.qi = self.parsed_mpd.get_qi()
        self.N = len(self.qi)
        self.send_up(msg)

    def handle_segment_size_request(self, msg):
        # time to define the segment quality choose to make the request
        msg.add_quality_id(self.qi[19]) #Aqui que colocamos a qualidade
        self.send_down(msg)

    def handle_segment_size_response(self, msg):
		# Mudar quando comecar a enviar outra qualidade pro 
		# msg.add_quality_id(self.qi[19])
        self.state = self.States(self.qi[19], 0) 
        print('State: ', self.state)
        #calcular recomensa
        #rodar o q-learning
        self.send_up(msg)

    def initialize(self):
        pass

    def finalization(self):
        pass
	
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
		
	##############################################################
	#########              Reward Functions              #########
	##############################################################
		
	# R_quality
    def reward_quality(self):
        r_quality = (self.qi-1)/(self.N-1)*2 - 1 #formula do artigo
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
        r_oscillation = 0 #definir ainda
        return r_oscillation
		
	# R_bufferfilling
    def reward_bufferfilling(self):
        r_bufferfilling = 0 #definir ainda
        return r_bufferfilling
	
	# R_bufferchange
    def reward_bufferchange(self):
        r_bufferchange = 0 #definir ainda
        return r_bufferchange
		
	# R - total reward
    def total_reward(self):
        r_quality = self.reward_quality()
        r_oscillation = self.reward_oscillation()
        r_bufferfilling = self.reward_bufferfilling()
        r_bufferchange = self.reward_bufferchange()
		
        reward = self.C1*r_quality + self.C2*r_oscillation + self.C3*r_bufferfilling + self.C4*r_bufferchange
        return reward
