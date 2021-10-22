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
	# quality that is going to be asked
        self.qi_request = 0
        self.last_policy_was_exploration = 0

    # buffers
        self.buffer_max = self.whiteboard.get_max_buffer_size()
        self.buffer_anterior = 1
        self.buffer_atual = 0

    #oscilação
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
        self.vector_qi = []
	
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

    # Vetores para criação de imagens
        self.vector_reward = []
        self.vector_exploration_explotation = []
        self.vector_time_reward = []
        self.vector_time_exploration_explotation = []

    #Pegando tabela Q do arquivo qtable.txt
        self.q_table = np.loadtxt('q_table_interval_25_profile_LLLMMMHMMMH_mudando_estado_inicial.txt')
        #print(self.q_table)


    def handle_xml_request(self, msg):
        self.request_time = time.time()
        self.send_down(msg)

    def handle_xml_response(self, msg):
        # getting qi list
        self.parsed_mpd = parse_mpd(msg.get_payload())
        self.qi = self.parsed_mpd.get_qi()
        self.N = len(self.qi)
		
        self.low_bandwidth = self.qi[0]
        self.low_medium_bandwidth = self.qi[floor(self.N/3) - 1]
        self.high_medium_bandwidth = self.qi[floor(2*self.N/3) - 1]
        self.high_bandwidth = self.qi[self.N - 1]
		
		# so consigo criar a tabela quando souber quantas qualidades tem
		# e as faixas de largura de banda
        #self.create_q_table()
		
        t = time.time() - self.request_time #diferenca de tempo
        self.bandwidth = msg.get_bit_length()/t #bits/s
        #print('Bandwidth: ', self.bandwidth)
		# mantem a qualidade e atualiza o bandwidth
        self.last_state = self.state
        self.state[1] = self.bandwidth
        #print('State: ', self.state)
		
        self.send_up(msg)

    def handle_segment_size_request(self, msg):
	
        self.request_time = time.time()

        if self.last_policy_was_exploration == 1:
            #print('Foi escolhido exploration, agora atualizando tabela Q: ')
            self.exploration_update_q_table()
		
        #############################
        # pedindo um valor aleatorio só pra pode testar
        #self.qi_id = random.randint(0, len(self.qi)-1)
        
        self.vector_qi.append(self.qi_request)

		# Quando for usar o q-learning colocar:
		#msg.add_quality_id(self.qi[self.qi_request])
        msg.add_quality_id(self.qi[self.qi_request]) #Aqui que colocamos a qualidade
        
        ##############

        # time to define the segment quality choose to make the request
        self.last_state = self.state
		# Quando for usar o q-learning colocar:
        self.state[0] = self.qi_atual
        #self.state[0] = self.qi_request 	# Mudando para o primeiro argumento ser de 0 
								# a 19 e n um valor em bps, era:
        #self.state[0] = self.qi[qi_id]

        #msg.add_quality_id(self.qi[19]) #Aqui que colocamos a qualidade

        self.send_down(msg)

    def handle_segment_size_response(self, msg):
        t = time.time() - self.request_time #diferenca de tempo
        self.bandwidth = msg.get_bit_length()/t #bits/s
        #print('Bandwidth: ', self.bandwidth)
		
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
			
        #print('State: ', self.state)
		
		# mantem a qualidade (que ainda sera alterada)
		# e atualiza o bandwidth
        if len(self.whiteboard.get_playback_buffer_size())!=0:
       	    self.buffer_anterior = self.buffer_atual
            self.buffer_atual = self.whiteboard.get_playback_buffer_size()[-1][1]

        #print(self.vector_qi)
		
        #if len(self.whiteboard.get_playback_history())!=0:
        #    print('        whiteboard.get_playback_history(): ',self.whiteboard.get_playback_history())

        if len(self.whiteboard.get_playback_qi())!=0:
            self.qi_anterior = self.qi_atual
            self.qi_atual = self.whiteboard.get_playback_qi()[-1][1]
        #    print('        whiteboard.get_playback_qi(): ',self.whiteboard.get_playback_qi())
        #    print('        >>>self.qi_atual: ',self.qi_atual)

        #print('QI anterior: ', self.qi_anterior)
        #print('QI atual: ', self.qi_atual)

		#Rodar o q-learning, chamando o e-greedy, que vai escolher entre exploration
		#e exploitation. Mas acho que tinhamos de dar um jeito de, no inicio explorar
		#pra ter algo na tabela Q...
		
        #Oq quero é um retorno de qual qualidade vou pedir na proxima request, ou
		#seja, quero atualizar o self.qi_request
        self.qi_request = self.e_greedy()
        #print('Egreedy: ', self.qi_request)
		
        #Eu acho que calcula a recompensa so se entrar em exploration...
		#Por ora podemos deixar so por questao de imprimir e ver oq ta rolando
		#calcular recomensa
        #print('Recompensa: ', self.total_reward())

        self.send_up(msg)

    def initialize(self):
    	pass

    def finalization(self):
        np.savetxt("q_table_interval_25_profile_LLLMMMHMMMH_mudando_estado_inicial.txt", self.q_table)
        self.graphic(self.vector_reward, self.vector_time_reward, 'reward', 'Reward', 'reward')
        self.graphic(self.vector_exploration_explotation, self.vector_time_exploration_explotation, 'explotation_exploration', 'Exploration (0.5) - Explotation (1.0)', 'Exploration/Explotation')

        

		
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
    #def create_q_table(self):
    #    self.q_table = np.zeros((5*self.N, self.N)) #5 faixas de bandwidth
	

	##############################################################
	#########              Reward Functions              #########
	##############################################################
		
	# R_quality
    def reward_quality(self):
        r_quality = (((self.qi_atual-1)/(self.N-1))*2) - 1 #formula do artigo
        return r_quality

	# R_oscillation
    def reward_oscillation(self):
        #Definindo se tem oscilação ou não com base na tendência de aumento ou decaimento
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
                #print('Depth:', self.depth)
        else:
            self.oscillation = 0
            self.length += 1

        #Calculando o valor da componente de recompensa dada a oscilação ou não
        if self.oscillation == 0:
            r_oscillation = 0
        else:
            if self.length >= self.max_length:
                r_oscillation = 0
            else:
                r_oscillation = -1/float(self.length)**(2/self.depth) + (float(self.length)-1)/((self.max_length-1)*self.max_length**(2/self.depth))
            self.length = 1

        #print('Oscilacao: ', r_oscillation)
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
        #print('Recompensa qualidade: ', r_quality)
        r_oscillation = self.reward_oscillation()
        #print('Recompensa oscilacao: ', r_oscillation)
        r_bufferfilling = self.reward_bufferfilling()
        #print('Recompensa bufferfilling: ', r_bufferfilling)
        r_bufferchange = self.reward_bufferchange()
        #print('Recompensa bufferchange: ', r_bufferchange)
		
        reward = self.C1*r_quality + self.C2*r_oscillation + self.C3*r_bufferfilling + self.C4*r_bufferchange
        #print('Recompensa total: ', reward)
        self.vector_reward.append(reward)
        self.vector_time_reward.append(time.time())
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
        epsilon = 0.8 	# Costuma ser bem baixo, tipo, 0.05, to colocando
						# maior pra ele explorar mais por ora
        random_number = np.random.random()
        if random_number < epsilon:
            self.last_policy_was_exploration = 1
            self.vector_exploration_explotation.append(0.5)
            self.vector_time_exploration_explotation.append(time.time())
            return self.exploration_choose_qi()
        else:
            self.last_policy_was_exploration = 0
            if (len(self.vector_time_reward) != 0):
                self.vector_reward.append(self.vector_reward[-1])
                self.vector_time_reward.append(time.time())
            else:
                self.vector_reward.append(0)
                self.vector_time_reward.append(time.time())
            
            self.vector_exploration_explotation.append(1.0)
            self.vector_time_exploration_explotation.append(time.time())
            return self.exploitation()
	
	######################## Exploration #########################
	
	# seleciona aleatoriamente o próximo estado e atualiza a 
	# tabela Q com o resultado de Q(s, a) = Q(s, a) + α [r + γ
	# max_b(s',b) - Q(s,a)] - Bellman Equation
	
    def exploration_choose_qi(self):
        #print('         ||>>> em exploration')
	
		#Qualidade que vou pedir na proxima requisicao
        random_number = random.randrange(0,self.N,1)
			
        #print('Qualidade escolhida: ', random_number)
        return random_number
		
    def exploration_update_q_table(self):
        #print('Atualizando a tabela Q')

		# Preciso ver qual estado eu tava, pegar qual o valor Q
		# atual desse estado na tabela Q, e fazer o calculo
		# da equacao de Bellman. Pra isso, preciso ter o estado 
		# anterior: self.last_state.
        indice1 = self.N*self.last_state[1] + self.last_state[0] #linha da tabela = N*BW + qi
        indice2 = self.qi_request #coluna da tabela = qualidade
        indice3 = self.N*self.state[1] + self.state[0] #linha da tabela = N*BW + qi
		# Dificuldade agora: termo max_b(s',b)
        self.q_table[indice1][indice2] = self.q_table[indice1][indice2]+self.alfa*(self.total_reward()+self.gama*np.max(self.q_table[indice3,:]) - self.q_table[indice1][indice2])

        


	######################## Exploitation ########################
	
    def exploitation(self):
        #print('         ||>>> em exploitation')
        indice1 = self.N*self.state[1] + self.state[0] #linha da tabela = N*BW + qi
        #indice2 = self.last_state[0] #coluna da tabela = qualidade

        #print('Tabela: \n')
        #for i in range(self.N*5):
        #    print(i, ':', self.q_table[i, :])

        #print('Estado: ', self.state, '   Linha da tabela: ', indice1)
		
        valor_maximo = np.max(self.q_table[indice1,:])
        indice = np.where(self.q_table[indice1,:] == valor_maximo)
        indice_coluna = int(indice[0][0])
        return indice_coluna









    ###################### Gerar Gráfico #########################
    def graphic(self, vector_inf, vector_time, file_name, title, y_axis, x_axis='execution time(s)'):

        plt.plot(vector_time, vector_inf, label=file_name)
        plt.xlabel(x_axis)
        plt.ylabel(y_axis)
        plt.title(title)

        plt.savefig(f'./results/{file_name}.png')
        plt.clf()
        plt.cla()
        plt.close()