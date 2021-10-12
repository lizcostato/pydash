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


class R2AQLearning(IR2A):

	##############################################################
	##### Realizacao das funcoes abstratas herdadas de IR2A  #####
	##############################################################
    def __init__(self, id):
        IR2A.__init__(self, id)
        self.parsed_mpd = ''
        self.qi = []

    def handle_xml_request(self, msg):
        self.send_down(msg)

    def handle_xml_response(self, msg):
        # getting qi list
        self.parsed_mpd = parse_mpd(msg.get_payload())
        self.qi = self.parsed_mpd.get_qi()

        self.send_up(msg)

    def handle_segment_size_request(self, msg):
        # time to define the segment quality choose to make the request
        msg.add_quality_id(self.qi[19])
        self.send_down(msg)

    def handle_segment_size_response(self, msg):
        self.send_up(msg)

    def initialize(self):
        pass

    def finalization(self):
        pass
		
	##############################################################
	#########              Reward Functions              #########
	##############################################################
	
	############# Constants used to define the reward ############
	# Wieghts C1-C4 - the values ​​used were obtained in the article
	C1 = 2
	C2 = 1
	C3 = 4
	C4 = 3
	# Number of quality levels
	N = 20 # obter do arquivo XML (ainda ver como)
	
	# R_quality
	def reward_quality(self):
		r_quality = (self.qi-1)/(N-1)*2 - 1 #formula do artigo
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
		
		reward = C1*r_quality + C2*r_oscillation + C3*r_bufferfilling + C4*r_bufferchange
		return reward
