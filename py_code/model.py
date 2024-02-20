#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from numpy import linalg as LA

# Model class fot the bird leg
class model(object):
	# constructor
	def __init__(self, points, angles_hip_joints, angles_ankle_joints):
		self.M = [[0 for j in range(4)] for i in range(8)]
		self.a = [0 for j in range(6)]
		self.alpha = [0 for j in range(6)] 
		self.d = [0 for j in range(6)] 
		self.theta = [0 for j in range(6)] 	
		self.pim =  [0 for j in range(6)] 	
		self.mip =  [0 for j in range(6)] 	
		self.psi_h = 0.0
		self.eta_h = 0.0
		self.psi_a = 0.0
		self.eta_a = 0.0
		# z-axes for rotations
		self.z0 = np.array([0,0,1]).T
		## vertical hip yaw
		self.z1 = self.z0
		## horiz hip roll
		self.z2 = np.array([1,0,0]).T
		## hip inclined joint
		self.z3 = np.array([0,1,0]).T
		## knee joint
		self.z4 = np.array([0,1,0]).T
		## ankle joint
		self.z5 = np.array([0,1,0]).T
		## foot joint
		self.z6 = np.array([0,1,0]).T
		## toe joint
		self.z7 = self.z6

		self.xA = np.array([0,0,0]).T
		self.yA = np.array([0,0,0]).T
		self.zA = np.array([0,0,0]).T
		
		self.updateDHKKWithStancePosition(points[0],points[1],points[2],points[3],points[4])
		self.updateDHKKWithOrientationAnglesHipAnkle(angles_hip_joints,angles_ankle_joints)	


	#################################################################
	# updateDHKKWithOrientationAnglesHipAnkle
	# inputs : orientation angles for the 2nd inclined hip joint and for the ankle joint
	# outputs : nothing (calculates DHKK parameters for storage inside class instance
	#################################################################	
	def updateDHKKWithOrientationAnglesHipAnkle(self,ang_hip,ang_ankle):
		self.psi_h = ang_hip[0] * math.pi / 180.0
		self.eta_h = ang_hip[1] * math.pi / 180.0
		self.psi_a = ang_ankle[0] * math.pi / 180.0
		self.eta_a = ang_ankle[1] * math.pi / 180.0	
			## angles of inclined hip joint, psi is horiz, eta is vertical
		self.z3 = 	np.array([math.sin(self.eta_h)*math.cos(self.psi_h),math.sin(self.eta_h)*math.sin(self.psi_h),math.cos(self.eta_h)]).T
			# ankle joint axis
		self.z5 = (math.sin(self.eta_a)*math.cos(self.psi_a))*self.xA + (math.sin(self.eta_a)*math.sin(self.psi_a))*self.yA + 	(math.cos(self.eta_a))*self.zA
		self.calcDHKK()
			
		#################################################################
		# updateDHKKWithStancePosition
		# inputs : 3D coord of hip, knee, angle and foot points
		# outputs : nothing (calculates DHKK parameters for storage inside class instance
		#################################################################	
	def updateDHKKWithStancePosition(self,H,K,A,F,T):	
			self.M[0] = H - H
			self.M[1] = self.M[0]
			self.M[2] = self.M[0]
			self.M[3] = self.M[0]
			self.M[4] = K - H	
			self.M[5] = A - H
			self.M[6] = F - H
			self.M[7] = T - H         
			KA = A - K
			HK = K - H
			self.xA = KA/LA.norm(KA)
			self.yA = np.cross(self.xA,HK)/ LA.norm(np.cross(self.xA,HK))
			self.zA = np.cross(self.xA,self.yA)
			# knee joint axis
			self.z4 = -self.yA
			# ankle joint axis
			self.z5 = (math.sin(self.eta_a)*math.cos(self.psi_a))*self.xA + (math.sin(self.eta_a)*math.sin(self.psi_a))*self.yA + (math.cos(self.eta_a))*self.zA
			# foot joint axis
			FT = T - F
			self.z6 = np.cross(self.z0,FT)/LA.norm(np.cross(self.z0,FT))
			# dummy
			self.z7 = self.z6 
			# call calc of DHKKS
			self.calcDHKK()
		
		#################################################################
		# calcDHKK
		# inputs : none
		# outputs : none
		# intern calculation of DHKK parameters
		#################################################################	
	def calcDHKK(self):
			z = np.array([self.z0,self.z1,self.z2,self.z3,self.z4,self.z5,self.z6,self.z7])

			x = [[0 for j in range(3)] for i in range(8)]
			for i in range(1,7):
				if LA.norm(np.cross(z[i],z[i+1])) < 0.001 :
					x[i] = np.cross(np.cross(z[i],self.M[i+1]-self.M[i]),z[i]) / LA.norm(np.cross(z[i],self.M[i+1]-self.M[i])) 
				else :
					if ( np.dot(np.cross(z[i],z[i+1]),self.M[i+1]-self.M[i]) < 0 ) :
						s = -1
					else :
						s = 1
					x[i] = 	s * np.cross(z[i],z[i+1]) / LA.norm(np.cross(z[i],z[i+1]))
			
			x[0] = [1,0,0]
			for i in range(1,7):
				self.a[i-1] = np.dot(self.M[i]-self.M[i-1],x[i-1])
				self.alpha[i-1] = math.atan2( np.dot(x[i-1],np.cross(z[i-1],z[i])), np.dot(z[i-1],z[i]))
				self.theta[i-1] = math.atan2( np.dot(z[i],np.cross(x[i-1],x[i])), np.dot(x[i-1],x[i]))
			
				if ( np.dot(np.cross(z[i-1],z[i]),self.M[i]-self.M[i-1]) < 0 ) :
					s1 = -1
				else :
					s1 = 1
				
				if ( np.dot(np.cross(z[i],z[i+1]),self.M[i+1]-self.M[i]) < 0 ) :
					s2 = -1
				else :
					s2 = 1
				
				if (LA.norm(np.cross(z[i-1],z[i]))) < 0.001:
					self.pim[i-1] = np.dot(self.M[i]-self.M[i-1],z[i])
				else :
					self.pim[i-1] = s1 * np.dot(np.cross(x[i-1],z[i-1]),self.M[i]-self.M[i-1]) / LA.norm(np.cross(z[i-1],z[i])) 
					
				if (LA.norm(np.cross(z[i],z[i+1]))) < 0.001:
					self.mip[i-1] = 0
				else:		
					self.mip[i-1] = -s2* np.dot(np.cross(x[i],z[i+1]),self.M[i+1]-self.M[i]) / LA.norm(np.cross(z[i],z[i+1])) 
					
				self.d[i-1] = self.pim[i-1] + self.mip[i-1]
				
		#################################################################
		# calc_KneeAnkleFootCoord
		# inputs: angle commands hip roll, hip incl , knee, ankle
		# outputs : 3D coordinates of knee, ankle and foot in hip coord. frame
		#################################################################
	def calc_KneeAnkleFootCoord(self,qHipRoll=0,qHipIncl=0,qKnee=0,qAnkle=0):
			# calculate matrices
			num_matrices = 6
			ncol = 4
			nrow = 4
			T = [[[0 for j in range(ncol)] for i in range(nrow)] for k in range(num_matrices)]
			q = [0,qHipRoll,qHipIncl,qKnee,qAnkle,0]
			q_cmd = [0 for j in range(6)] 
			for i in range(num_matrices):
				q_cmd[i] =  self.theta[i] + q[i]
				T[i] = np.array( [[math.cos(q_cmd[i]),-math.sin(q_cmd[i]),0,self.a[i]],[math.cos(self.alpha[i])*math.sin(q_cmd[i]), math.cos(self.alpha[i])*math.cos(q_cmd[i]),-math.sin(self.alpha[i]),-self.d[i]*math.sin(self.alpha[i])],[math.sin(self.alpha[i])*math.sin(q_cmd[i]), math.sin(self.alpha[i])*math.cos(q_cmd[i]),		 math.cos(self.alpha[i]), self.d[i]*math.cos(self.alpha[i])], [0,0,0,1]])
			
			TH = np.dot(np.dot(T[0],T[1]),T[2])
			pt = np.array([[0,0,-self.mip[3],1]]).T
			TK = np.dot(TH,T[3])
			K_Rhip = np.dot(TK,pt)

			pt = np.array([[0,0,-self.mip[4],1]]).T
			TA = np.dot(TK,T[4])
			A_Rhip = np.dot(TA,pt)

			pt = np.array([[0,0,-self.mip[5],1]]).T
			TF = np.dot(TA,T[5])
			F_Rhip = np.dot(TF,pt)
		
			return K_Rhip, A_Rhip, F_Rhip


		#################################################################
		# calc_JacobianKneeAnkleFoot
		# inouts : current angles hip roll, hip incl , knee, ankle
		# outputs : Jacobian matrices for knee, ankle and foot
		#################################################################	
	def calc_JacobianKneeAnkleFoot(self,qHipRoll_prev=0,qHipIncl_prev=9,qKnee_prev=9,qAnkle_prev=0):
			#print('inside calc jac')
			num_matrices = 6
			ncol = 4
			nrow = 4
			qcurrent = [0,qHipRoll_prev,qHipIncl_prev,qKnee_prev,qAnkle_prev,0]
			#print('after qcurrent')
			q_cmd = [0 for j in range(6)] 
			DT = [[[0 for j in range(ncol)] for i in range(nrow)] for k in range(num_matrices)]
			T = [[[0 for j in range(ncol)] for i in range(nrow)] for k in range(num_matrices)]
			for i in range(num_matrices):
				q_cmd[i] =  self.theta[i] + qcurrent[i]
				DT[i] = np.array( [[-math.sin(q_cmd[i]),-math.cos(q_cmd[i]),0,0],[math.cos(self.alpha[i])*math.cos(q_cmd[i]), -math.cos(self.alpha[i])*math.sin(q_cmd[i]),0,0],[math.sin(self.alpha[i])*math.cos(q_cmd[i]), -math.sin(self.alpha[i])*math.sin(q_cmd[i]),0,0], [0,0,0,0]])
				T[i] = np.array( [[math.cos(q_cmd[i]),-math.sin(q_cmd[i]),0,self.a[i]],[math.cos(self.alpha[i])*math.sin(q_cmd[i]), math.cos(self.alpha[i])*math.cos(q_cmd[i]),-math.sin(self.alpha[i]),-self.d[i]*math.sin(self.alpha[i])],[math.sin(self.alpha[i])*math.sin(q_cmd[i]), math.sin(self.alpha[i])*math.cos(q_cmd[i]),		 math.cos(self.alpha[i]), self.d[i]*math.cos(self.alpha[i])], [0,0,0,1]])

			#print('before pt')
			pt = np.array([0,0,-self.mip[3],1])
			MJq1 = np.dot(np.dot(np.dot(T[0],DT[1]),T[2]),T[3])
			MJq2 = np.dot(np.dot(np.dot(T[0],T[1]),DT[2]),T[3])
			JKnee_q1 =  np.dot(MJq1,pt)
			JKnee_q2 =  np.dot(MJq2,pt)
			JKnee = np.array([JKnee_q1[0:3],JKnee_q2[0:3]]).T
			#print(JKnee)

			pt = np.array([0,0,-self.mip[4],1])
			MAq1 = np.dot(MJq1,T[4])
			MAq2 = np.dot(MJq2,T[4])
			AInter = np.dot(np.dot(np.dot(T[0],T[1]),T[2]),DT[3])
			MAq3 = np.dot(AInter,T[4])
			JAnkle_q1 =  np.dot(MAq1,pt)
			JAnkle_q2 =  np.dot(MAq2,pt)
			JAnkle_q3 =  np.dot(MAq3,pt)
			JAnkle = np.array([JAnkle_q1[0:3],JAnkle_q2[0:3],JAnkle_q3[0:3]]).T
			#print(JAnkle)
			
			pt = np.array([0,0,-self.mip[5],1])
			MFq1 = np.dot(MAq1,T[5])
			MFq2 = np.dot(MAq2,T[5])
			MFq3 = np.dot(MAq3,T[5])
			Inter1 = np.dot(np.dot(np.dot(T[0],T[1]),T[2]),T[3])
			FInter = np.dot(Inter1,DT[4])
			MFq4 = np.dot(FInter,T[5])
			JFoot_q1 =  np.dot(MFq1,pt)
			JFoot_q2 =  np.dot(MFq2,pt)
			JFoot_q3 =  np.dot(MFq3,pt)
			JFoot_q4 =  np.dot(MFq4,pt)
			JFoot = np.array([JFoot_q1[0:3],JFoot_q2[0:3],JFoot_q3[0:3],JFoot_q4[0:3]]).T
			#print(JFoot)
			
			return JKnee, JAnkle, JFoot
