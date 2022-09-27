#!/usr/bin/env python
import rospy
import serial
from time import sleep
import roslib; roslib.load_manifest('sound_play')
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Int32
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Byte
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ugv_msgs.msg import Node
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
from sensor_msgs.msg import BatteryState
import minimalmodbus
import threading
from threading import Timer, Thread, Event
import socket
import struct
from DEFINE import *
from sensor_msgs.msg import BatteryState

class QQTimer():
	def __init__(self,t,hFunction):
		self.t = t
		self.hFunction = hFunction
		self.thread = Timer(self.t,self.handle_function)
	def handle_function(self):
		self.hFunction()
		self.thread = Timer(self.t,self.handle_function)
		self.thread.start()
	def start(self):
		self.thread.start()
	def stop(self):
		self.thread.cancel()

#=============================================================================================#

# ================================= GIAO TIEP VOI HOST PC ====================================#

#========================================================================
#=======================================================================
#def getBAT(msg):
#	global BAT
#	BAT = msg
#	print "charging:", BAT.charge, "%:", BAT.percentage
	
def sendSpeedCmd():
	global ser
	global left
	global right
	data = b'!M {0} {1}\r'
	ser.write(data.format(left,right))
	#print "cmd:", left, right

# def getUD(msg):
# 	global LIFT_UP
# 	global LIFT_DOWN
# 	global pose_ht
# 	if msg.data==1: # NANG
# 	    LIFT_UP = 1
# 	    LIFT_DOWN=0
# 	    pose_ht.orientation.z = 1
# 	if msg.data==0:
# 	    LIFT_DOWN=1
# 	    LIFT_UP=0
# 	    pose_ht.orientation.z = 0

def PLC_rw():
	global Front_Obs
	global Back_Obs
	global addr_Front_Obs
	global addr_Back_Obs
	global Bumper_Front
	global Bumper_Back
	global addr_Bumper_Front
	global addr_Bumper_Back	
	global PLC
	global addr_Lift_Up
	global addr_Lift_Dn
	global addr_Btn_Present
	global complete
	global addr_run
	global chaytiep
	global cnt
	global LIFT_UP
	global LIFT_DOWN
	global ck_connection
	global addr_dk_motor
	global ck
	global addr_ID
	global addr_Mang_xe
	global addr_NV
	global addr_Hoanthanh_NV
	global addr_DK
	global addr_OLD_ID
	global addr_Estop      	# EMERGENCY signal
	global Estop
	global jobFinished
	global vibot_state
	global NV
	global is_reset
	global addr_PC_treo
	global addr_reset_cam
	global cmd_reset_cam
	global PC_treo
	global pose_ht
	global auto_on
	global bumper_front_reset
	global bumper_back_reset
	if (is_reset==1):
		if ck==0:
			try:
				# Doc vat can truoc/sau
				if (cnt % 2 == 0):
					Front_Obs = PLC.read_bit(addr_Front_Obs)
				elif cnt==1:
					Back_Obs = PLC.read_bit(addr_Back_Obs)
				# Doc trang thai nut bam
				elif cnt==3:
					complete = PLC.read_bit(addr_Btn_Present)
					if complete==1 and jobFinished == 1:
						PLC.write_bit(addr_Btn_Present,0,5)
				# Ghi lenh NANG, HA xe
				elif cnt==5:
					PLC.write_bit(addr_Lift_Up,LIFT_UP,5)
				elif cnt==7:
					PLC.write_bit(addr_Lift_Dn,LIFT_DOWN,5)
				# Ghi trang thai den
				elif cnt==9:
					if auto_on==1:
						PLC.write_bit(addr_run,chaytiep,5)
					else:
						PLC.write_bit(addr_run,1-chaytiep, 5)
				elif cnt==11:
					ck = PLC.read_bit(addr_dk_motor)
				# Ghi trang thai robot len vung nho cua PLC
				#pose_ht.position.x=FULL_PATH[PLC.read_register(addr_ID,1)-3]
				#pose_ht.position.y=PLC.read_register(addr_NV,1)
				#pose_ht.position.z=PLC.read_register(addr_DK,1)
				#pose_ht.orientation.x = PLC.read_register(addr_OLD_ID,1)
				#pose_ht.orientation.y = PLC.read_register(addr_Hoanthanh_NV,1)
				#pose_ht.orientation.z=PLC.read_register(addr_Mang_xe,1)

				elif cnt==13: # VT hien tai
					if (pose_ht.position.x>0):
						value = int(getINDEX(pose_ht.position.x))
						PLC.write_register(addr_ID,value,1)
				elif cnt==15: # mang xe
					PLC.write_register(addr_Mang_xe,int(pose_ht.orientation.z),1)
				elif cnt==17: # NV
					PLC.write_register(addr_NV,int(pose_ht.position.y),1)
				elif cnt==19: # VT hoan thanh NV
					PLC.write_register(addr_Hoanthanh_NV,int(pose_ht.orientation.x),1)
				elif cnt==21: # DK
					PLC.write_register(addr_DK,int(pose_ht.position.z),1)
				elif cnt==23: # OLD_ID
					value = int(getINDEX(pose_ht.orientation.y))
					PLC.write_register(addr_OLD_ID,value,1)

				elif cnt==25: # Reset lai CAM
					PLC.write_bit(addr_reset_cam,cmd_reset_cam,5)
					cmd_reset_cam = 0
				elif cnt == 27:
					PC_treo = 1-PC_treo
					PLC.write_bit(addr_PC_treo,PC_treo,5)
				elif cnt == 29:
					Bumper_Front = PLC.read_bit(addr_Bumper_Front)
				elif cnt == 31:
					Bumper_Back = PLC.read_bit(addr_Bumper_Back)
				elif cnt == 33:
					if bumper_front_reset:
						PLC.write_bit(addr_Bumper_Front,0,5)
						bumper_front_reset = 0
				elif cnt == 35:
					if bumper_back_reset:
						PLC.write_bit(addr_Bumper_Back,0,5)
						bumper_back_reset = 0	
				elif cnt == 37:
					Estop = PLC.read_bit(addr_Estop)	
			except :
				print "PLC_rw: ID:", getINDEX(pose_ht.position.x)-3,"\nOLD_ID:", getINDEX(pose_ht.orientation.y)-3 , "\nMang_xe:", pose_ht.orientation.z, "\nNV:", pose_ht.position.y, "\njobFinished:", pose_ht.orientation.x,"\nDK:", pose_ht.position.z
				# Thiet lap ket noi voi PLC
				PLC = minimalmodbus.Instrument('/dev/ttyS0',5)  # port name, slave address (in decimal)
				PLC.serial.baudrate = 115200         # Baud
				PLC.serial.parity   = minimalmodbus.serial.PARITY_EVEN
				PLC.serial.timeout  = 5          # seconds0
				PLC.mode = minimalmodbus.MODE_RTU   # rtu or ascii mode MODE_ASCII, MODE_RTU
				print "error r/w PLC"
				pass
		else:
			ck_connection = ck_connection+1
			if ck_connection>200:
				ck_connection=0
			ck = 0
		cnt = cnt+1
		#print ck
		if cnt==38:
			cnt = 0

def bumper_reset():
	global bumper_back_reset, bumper_front_reset
	global dx
	if dx > 0: # TIEN
		bumper_back_reset = 1
	if dx < 0: # LUI
		bumper_front_reset = 1		
def reset():
	global addr_ID
	global addr_Mang_xe
	global addr_NV
	global addr_Hoanthanh_NV
	global addr_DIR
	global addr_OLD_ID
	global PLC
	global FULL_PATH
	global is_reset
	global is_mangxe
	global jobFinished
	global pose_ht
	global next_ID
	global FULL_PATH
	is_reset = 0
	pose_ht.position.x=FULL_PATH[int(PLC.read_register(addr_ID,1))]
	pose_ht.position.y=PLC.read_register(addr_NV,1)
	pose_ht.position.z=PLC.read_register(addr_DK,1)
	pose_ht.orientation.y = FULL_PATH[int(PLC.read_register(addr_OLD_ID,1))]
	pose_ht.orientation.x = PLC.read_register(addr_Hoanthanh_NV,1)
	pose_ht.orientation.z=PLC.read_register(addr_Mang_xe,1)
	if (getINDEX(pose_ht.orientation.y) > getINDEX(pose_ht.position.x)):
		if (getINDEX(pose_ht.position.x)>0): 
			next_ID = FULL_PATH[getINDEX(pose_ht.position.x)-1]
		else: # dang o tram sac
			next_ID = FULL_PATH[0]
	else:				
		next_ID = FULL_PATH[getINDEX(pose_ht.position.x)+1]
	if DEBUG_TRAMSAC:
		pose_ht.position.x = FULL_PATH[0]
		print "DEBUG_SAC"
	print "is_reset:", is_reset
	print "ID:", getINDEX(pose_ht.position.x)-3,"\nOLD_ID:", getINDEX(pose_ht.orientation.y)-3 , "\nMang_xe:", pose_ht.orientation.z, "\nNV:", pose_ht.position.y, "\njobFinished:", pose_ht.orientation.x,"\nDK:", pose_ht.position.z
	is_reset = 1
#===========================================================================

def sign(x):
	if x>0:
		return 1
	elif x==0:
		return 0
	elif x<0:
		return -1

def getRFID(msg):
	global VT_hientai
	global pose_ht
	global passed_rfid
	global vibot_state
	global quaylai_ok
	global Sac
	global auto_on
	global old_msg
	global N_ID
	global next_ID
	#print "RFID"
	if (old_msg != msg.data):
		old_msg = msg.data
		if (msg.data > 0) and (getINDEX(msg.data)!=-1):
			print getINDEX(msg.data)-3," ",  msg.data
			if (VT_hientai.ID==0):
				if (len(TaskList)>0):					
					if (len(TaskList)>1):
						next_ID = TaskList[1].ID
						if (TaskList[1].ID==msg.data):
							del TaskList[0]
					if (TaskList[0].ID==msg.data):
						VT_hientai.ID = TaskList[0].ID
						VT_hientai.NV = TaskList[0].NV
						VT_hientai.DK = TaskList[0].DK
						VT_hientai.DIR = TaskList[0].DIR
						pose_ht.orientation.y = pose_ht.position.x  # Luu OLD ID
						pose_ht.position.x = TaskList[0].ID	    # NEW ID
						pose_ht.position.y = TaskList[0].DK	    # NEW DK
						pose_ht.position.z = TaskList[0].NV	    # NEW NV

						#pose_ht.orientation.x = jo	    # NEW DIR
						#if (msg.data == Sac):
						#     pose_ht.position.x = 0  # Quy uoc tai vi tri Sac, ID = 0 (thong nhat voi ch. trinh tren HOST PC)
						print "CURRENT TASK: P.",getINDEX(VT_hientai.ID)-3, "DK:", VT_hientai.DK, "NV:", VT_hientai.NV, "DIR:", VT_hientai.DIR
			if ((auto_on==0) or (auto_on==2)) and (pose_ht.position.x != msg.data): # Dang o che do bang tay
				pose_ht.orientation.y = pose_ht.position.x
				pose_ht.position.x = msg.data # Chi ghi ID hien tai			
				if (auto_on==2):
					N_ID = N_ID+1
				
				
def getVachTu(msg):
	global dm
	dm = 7.5-msg.data

#===================================== DIEU KHIEN BANG TAY ==============================#
def MANUAL_JOY(msg):
	global msg_joy, msg_joy_update
	msg_joy = msg
	msg_joy_update=1
	if DEBUG_JOY:
		print msg

#========================================================================================#
UDP_IP_SEND = "172.20.10.3"
UDP_IP_READ = "172.20.10.145"
UDP_IP_FROM = "172.20.10.3"

UDP_PORT_READ_PATH = 10000
UDP_PORT_SEND = 10001
UDP_PORT_READ_PC = 10002
UDP_PORT_READ_RESET = 10003
UDP_PORT_READ_FULL = 10004

def INIT_UDP():
	global sock_read_reset
	global sock_read_pc
	global sock_read_FULL
	global sock_read_path
	global sock_send_feedback	
	
	ok = 1
	while ok:
		try:
			sock_read_reset = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
			sock_read_reset.bind((UDP_IP_READ, UDP_PORT_READ_RESET))

			sock_read_pc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
			sock_read_pc.bind((UDP_IP_READ, UDP_PORT_READ_PC))
			
			sock_read_FULL = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
			sock_read_FULL.bind((UDP_IP_READ, UDP_PORT_READ_FULL))
			
			sock_read_path = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
			sock_read_path.bind((UDP_IP_READ, UDP_PORT_READ_PATH))
		
			sock_send_feedback = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

			ok = 0
			print "Ket noi thanh cong"
			threading.Thread(target=MANUAL_PC).start()
			threading.Thread(target=MANUAL_RESET).start()
			threading.Thread(target=MANUAL_FULL_PC).start()
			threading.Thread(target=AUTO_GET_PATH).start()
		except:
			ok = 1
			print "Chua co ket noi"
			sleep(1)
	

def MANUAL_PC():
	global msg_manual_pc
	global msg_manual_pc_update, sock_read_pc
	while True:
		try:
			msg_manual_pc, addr_man = sock_read_pc.recvfrom(1024)
			if (addr_man[0] == UDP_IP_FROM):
				msg_manual_pc_update=1
		except:
			print "mat ket noi..."
			sleep(2)
		#if DEBUG_MANUAL: print msg_manual_pc
#threading.Thread(target=MANUAL_PC).start()

#==========================================================================================#
def MANUAL_RESET():
	global msg_reset, msg_reset_update, sock_read_reset
	while True:
		try:
			msg_reset, rpose_addr = sock_read_reset.recvfrom(1024)
			if (rpose_addr[0] == UDP_IP_FROM):
				msg_reset_update = 1
		except:
			print "mat ket noi..."
			sleep(2)
#threading.Thread(target=MANUAL_RESET).start()
#======================= LAY CH TRIMNH FULL ===========================#

def MANUAL_FULL_PC():
	global msg_full_pc
	global msg_full_pc_update, sock_read_FULL
	while True:
		try:
			msg_full_pc, rpose_addr = sock_read_FULL.recvfrom(1024)
			if (rpose_addr[0] == UDP_IP_FROM):
				msg_full_pc_update = 1
		except:
			print "mat ket noi..."
			sleep(2)
		#if DEBUG_MANUAL: print msg_full_pc
#threading.Thread(target=MANUAL_FULL_PC).start()

def AUTO_GET_PATH():
	global msg_auto_path, sock_read_path
	global posepatharray, get_cmd_path_update
	global gotTask
	global pose_ht
	global started
	global chaytiep
	global vibot_state
	global XuatPhat
	global NV
	global FULL_PATH
	global PHONG
	global VT_hientai
	global old_msg
	global TaskList
	global V
	while True:
		try:
			print "auto get path"
			msg_auto_path, addr = sock_read_path.recvfrom(4096)
			if (addr[0]==UDP_IP_SEND):	
				posepatharray = PoseArray()
				#print data
				for i in range(1,len(msg_auto_path), 12):
					tmp_rfid = msg_auto_path[i:i+10]
					tmp_direction = msg_auto_path[i+10]
					tmp_task = msg_auto_path[i+11]
					#print tmp_rfid, tmp_direction, tmp_task 
					posepath = Pose()
					posepath.position.x = int(tmp_rfid)
					posepath.position.y = int(tmp_direction)
					posepath.position.z = int(tmp_task)
					posepatharray.poses.append(posepath)
				#======================== XU LY DU LIEU =======================================#	
				if gotTask == 0:
					for i in range(0,len(posepatharray.poses)):
						TaskList.append(Node(posepatharray.poses[i].position.x,posepatharray.poses[i].position.y,posepatharray.poses[i].position.z,0))
					N = len(TaskList)
					print "---------------------------------------------------"
					for i in range(0,N):
						print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV, TaskList[i].DIR
					if (TaskList[i].NV < 5) and (TaskList[i].NV>0):
						NV = TaskList[i].NV
					gotTask = 1
					if len(TaskList)>1: # co it nhat 2 diem tren quy dao, tinh ca diem robot dang dung
						chieu = sign(getINDEX(TaskList[1].ID)-getINDEX(TaskList[0].ID))
						print "chieu:", chieu, started
					else:
						chieu = 0
						print "chieu:", chieu
					if (chieu!=0):
					# XU LY CHIEU DI CUA ROBOT
						N = len(TaskList)
						for i in range(0,N):
							print getINDEX(TaskList[i].ID)-3
					   # print FULL_PATH[i]
						for i in range(0,N-1):
							TaskList[i].DIR = sign(getINDEX(TaskList[i+1].ID)-getINDEX(TaskList[i].ID))
							#print "ck:", i+1, getINDEX(TaskList[i+1].ID), getINDEX(TaskList[i].ID)
						TaskList[-1].DIR = TaskList[-2].DIR
						pose_ht.orientation.x = 0

						print "TASK LIST"
						N = len(TaskList)
						for i in range(0,N):
							print getINDEX(TaskList[i].ID)-3, TaskList[i].DK, TaskList[i].NV, TaskList[i].DIR

						print "NEW TASK: ID: ", VT_hientai.ID, "NV: ", VT_hientai.NV, "DK: ", VT_hientai.DK
						chaytiep = 1
						#V = 0
						V = 0.05
						W = 0
						dI = 0
						VT_hientai.ID = TaskList[0].ID
						VT_hientai.NV = TaskList[0].NV
						VT_hientai.DK = TaskList[0].DK
						VT_hientai.DIR = TaskList[0].DIR
						#pose_ht.orientation.y = pose_ht.position.x  # Luu OLD ID
							#pose_ht.position.x = TaskList[0].ID	    # NEW ID
						pose_ht.position.y = TaskList[0].DK	    # NEW DK
						pose_ht.position.z = TaskList[0].NV	    # NEW NV
						print "NEW TASK: ID: ", getINDEX(VT_hientai.ID)-3, "NV: ", VT_hientai.NV, "DK: ", VT_hientai.DK
						old_msg = -1
					else:
						del TaskList[:]
						gotTask=0
		except:
			print "mat ket noi..."
			sleep(2)


def getBAT(msg):
	global BAT
	BAT = msg

def AUTO_SEND_FEEDBACK():
	global sock_send_feedback, pose_ht
	global check, sock_send, BAT, msg_status
	global BAT
	pose_send = []
	pose_send += '@' + '{0:010d}'.format(int(pose_ht.position.x)) + str(int(pose_ht.position.y)) + str(int(pose_ht.position.z))
	pose_send += str(int(pose_ht.orientation.z)) +'{0:010d}'.format(int(pose_ht.orientation.y))+'{0:010d}'.format(int(pose_ht.orientation.x))+'{0:010d}'.format(int(BAT.percentage))+msg_status
	#print pose_send
	try:		
		sock_send_feedback.sendto(''.join(pose_send), (UDP_IP_SEND, UDP_PORT_SEND))		
	except:
		print "mat ket noi..."
		sleep(2)
	  
	
def MANUAL_BANG_TAY():
	global msg_manual_pc,msg_joy, msg_full_pc, msg_reset
	global msg_manual_pc_update, msg_joy_update, msg_full_pc_update, msg_reset_update
	global auto_on, N_ID
	global dr, dx, gotTask, pose_ht, LIFT_UP, LIFT_DOWN, pose_ht, complete
	a_scale_ = 0.4
	l_scale_ = 0.8
	angular_ = 2
	linear_ = 3
	msg_manual_pc_update = 0
	msg_joy_update = 0
	msg_full_pc_update = 0
	msg_reset_update = 0
	load = 0
	ck_auto = 0
	ck_manual = 0
	#ck_auto_track = 0	
	if DEBUG_MANUAL: print "MANUAL_BANGTAY"
	while True:
		if msg_manual_pc_update: # xu ly lenh PC UDP manual
			msg_manual_pc_update=0
			#if DEBUG_MANUAL: print "M-PC", msg_manual_pc
			if (msg_manual_pc[0]=='1') and (ck_auto == 0):
				ck_auto = 1
				ck_manual = 0
				#ck_auto_track = 0
				INIT_JOB()
				if DEBUG_MANUAL: print "auto"
			elif (msg_manual_pc[0]=='2') and (ck_manual==0):
				ck_manual = 1
				ck_auto = 0
				#ck_auto_track = 0
				auto_on = 0
				if DEBUG_MANUAL: print "manual"
			elif (msg_manual_pc[0]=='S'): # Tu dong do duong
				#ck_auto_track = 1
				ck_auto = 0
				ck_manual = 0				
				auto_on = 2	
				N_ID = 0			
				if DEBUG_MANUAL: print "auto_following_line"
			if (auto_on==1):  # xu ly nut bam tu dong ket thuc nhiem vu
				if (msg_manual_pc[0]=='T') and (complete==0):
					complete = 1
			if (auto_on==0):	
				# DK NANG/HA XE			
				if (msg_manual_pc[0]=='3'):
					LIFT_UP = 1  #---- NANG
					LIFT_DOWN = 0
					pose_ht.orientation.z = 1
				elif (msg_manual_pc[0]=='4'):
					LIFT_UP = 0  #---- HA
					LIFT_DOWN = 1
					pose_ht.orientation.z = 0
				# DK CHUYEN DONG CUA ROBOT
				elif (msg_manual_pc[0]=='5'): #---- DUNG LAI
					dr = 0
					dx = 0
				elif (msg_manual_pc[0]=='6'): #---- TIEN						
					dx = 0.1
					dr = 0
					bumper_reset()
				elif (msg_manual_pc[0]=='7'): #---- LUI
					dx = -0.1
					dr = 0
					bumper_reset()
				elif (msg_manual_pc[0]=='8'): #---- QUAY TRAI
					dx = 0
					dr = 0.2
				elif (msg_manual_pc[0]=='9'): #---- QUAY PHAI
					dx = 0
					dr = -0.2
								
			else:
				if (gotTask == 0) and (pose_ht.position.x == FULL_PATH[0]):
					load = 0
					if (msg_manual_pc[0]=='A'):
						CT_DuaCom_FULL()
						load = 1
					elif (msg_manual_pc[0]=='B'):
						CT_DuaThuoc_FULL()
						load = 1
					elif (msg_manual_pc[0]=='C'):
						CT_ThamKham_FULL()
						load = 1
					elif (msg_manual_pc[0]=='D'):
						CT_LayRac_FULL()
						load = 1
					if load:
						VT_hientai.ID = TaskList[0].ID
						VT_hientai.NV = TaskList[0].NV
						VT_hientai.DK = TaskList[0].DK
						VT_hientai.DIR = TaskList[0].DIR

		if msg_joy_update:
			msg_joy_update=0
			#if DEBUG_MANUAL: print msg_joy
			if msg_joy.buttons[3]:
				INIT_JOB()
				if DEBUG_MANUAL: print "auto"
			elif msg_joy.buttons[1]:
				auto_on = 0
				if DEBUG_MANUAL: print "manual"
			elif msg_joy.buttons[2]:
				auto_on = 2
				N_ID = 0
				if DEBUG_MANUAL: print "auto_track_line"
			if (auto_on==0):
				dr = a_scale_ * msg_joy.axes[angular_]
				dx = l_scale_ * msg_joy.axes[linear_]
				bumper_reset()
				
				if msg_joy.axes[1]>0:
					LIFT_UP = 1  # NANG
					LIFT_DOWN = 0
				elif msg_joy.axes[1]<0:
					LIFT_UP = 0  # HA
					LIFT_DOWN = 1
			else:
				if (gotTask == 0) and (pose_ht.position.x == FULL_PATH[0]):
					load = 0
					if msg_joy.buttons[4]:
						INIT_JOB()
						CT_DuaCom_FULL()
						load = 1
					elif msg_joy.buttons[5]:
						INIT_JOB()
						CT_DuaThuoc_FULL()
						load = 1
					elif msg_joy.buttons[6]:
						INIT_JOB()
						CT_ThamKham_FULL()
						load = 1
					elif msg_joy.buttons[7]:
						INIT_JOB()
						CT_LayRac_FULL()
						load = 1
					if load:
						VT_hientai.ID = TaskList[0].ID
						VT_hientai.NV = TaskList[0].NV
						VT_hientai.DK = TaskList[0].DK
						VT_hientai.DIR = TaskList[0].DIR
		if msg_full_pc_update:
			msg_full_pc_update=0
			if DEBUG_MANUAL: print "F-PC:", msg_full_pc
			full_auto = msg_full_pc[0]
			full_CT = msg_full_pc[1:len(msg_full_pc)]
			if (auto_on==1) and (gotTask == 0) and (pose_ht.position.x == FULL_PATH[0]):  # Co NV moi va dang o tram sac
				load = 0
				if (full_CT == 'Com1'):
					CT_DuaCom_FULL()
					load = 1
				elif (full_CT == 'Thuoc1'):
					CT_DuaThuoc_FULL()
					load = 1
				elif (full_CT == 'ThamKham1'):
					CT_ThamKham_FULL()
					load = 1
				elif (full_CT == 'Rac1'):
					CT_LayRac_FULL()
					load = 1
				if load:
					VT_hientai.ID = TaskList[0].ID
					VT_hientai.NV = TaskList[0].NV
					VT_hientai.DK = TaskList[0].DK
					VT_hientai.DIR = TaskList[0].DIR

threading.Thread(target=MANUAL_BANG_TAY).start()

#==========================================================================================#
def getEncoder():	# doc gia tri encoder cuar hai dong co
	global ser
	global lenc
	global renc
	ser.write(b'?C\r')
	while True:
		data = ser.read_until('\r',None)
		if data.startswith('C='):
			ser.reset_input_buffer()
			enc = data.strip().replace('C=','').split(':')
			lenc = int(enc[0])
			renc = int(enc[1])
			#print "enc:", lenc, renc
			break
#==========================================================================================================
#==================================  JOB PROCESS ==========================================================
def jobProcess2():	# Thuc hien nhiem vu
	global jobStart
	global VT_hientai
	global layxeID
	if jobStart == 1:
		if VT_hientai.NV==1:
			DUA_COM()
		elif VT_hientai.NV==2:
			DUA_THUOC()
		elif VT_hientai.NV==3:
			THAM_KHAM()
		elif VT_hientai.NV==4:
			LAY_RAC()
		elif VT_hientai.NV==5:
			LAY_XE()
		elif VT_hientai.NV==6:
			TRA_XE()

def LAY_XE():
	global count
	global sound_client
	global complete
	global pose_ht
	global VT_hientai
	global jobFinished
	global NV
	global complete
	global LIFT_UP
	global LIFT_DOWN
	if (count==0):
		if (NV==1):	# Lay xe com
			sound_client.playWave("/opt/ros/melodic/share/sound_play/sounds/xinmoi_duaxecomvao.wav")
		elif (NV==2): # Lay xe thuoc
			sound_client.playWave("/opt/ros/melodic/share/sound_play/sounds/xinmoi_duaxethuocvao.wav")
		elif (NV==4): # lay xe rac
			sound_client.playWave("/opt/ros/melodic/share/sound_play/sounds/xinmoi_duaxeracvao.wav")
				#if (PLC.read_bit(addr_load)==1):
		LIFT_UP=1
		LIFT_DOWN=0
		pose_ht.orientation.z = 1 # Da lay xe

	if ((count>200) and (complete==1)):
		count = 0
		jobFinished=1
		jobStart = 0
		pose_ht.orientation.x=1		# DA HOAN THANH NV
		VT_hientai.NV=0;
		complete = 0
		LIFT_UP = 0
		sound_client.playWave("/opt/ros/melodic/share/sound_play/sounds/xincamon_tambiet.wav")
		print "Thuc hien XONG NV"
		print "Job:", jobFinished,"DK:", VT_hientai.DK, "Run:", chaytiep
	count = count+1

def TRA_XE():
	global count
	global sound_client
	global complete
	global pose_ht
	global VT_hientai
	global jobFinished
	global complete
	global LIFT_DOWN
	global LIFT_UP
	if (count==0):
		sound_client.playWave("/opt/ros/melodic/share/sound_play/sounds/xinmoi_layxera.wav")
		LIFT_DOWN = 1
		LIFT_UP=0
		pose_ht.orientation.z = 0   	# DA TRA XE
	if ((count>200) and (complete==1)):
		count = 0
		jobFinished=1
		jobStart = 0
		pose_ht.orientation.x=1		# DA HOAN THANH NV
		VT_hientai.NV=0;
		LIFT_DOWN=0
		complete = 0
		sound_client.playWave("/opt/ros/melodic/share/sound_play/sounds/xincamon_tambiet.wav")
		print "Thuc hien XONG NV"
		print "Job:", jobFinished,"DK:", VT_hientai.DK, "Run:", chaytiep
	count = count + 1

def DUA_COM():
	global count
	global VT_hientai
	global sound_client
	global jobFinished
	global jobStart
	global pose_ht
	global complete

	# Doc trang thai nui nhan Complete
	if count ==0:
		sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/xinmoi_laycom.wav')
	if (count >200) and (complete==1): #job is finished, chuyen tiep den cong viec tiep theo
		count = 0
		jobFinished = 1
		jobStart = 0
		pose_ht.orientation.x=1		# DA HOAN THANH NV
		VT_hientai.NV = 0 	# Da lam xong Task
		complete = 0
		sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/xincamon_chucngonmieng.wav')
		print "Thuc hien XONG NV"
		print "Job:", jobFinished,"DK:", VT_hientai.DK, "Run:", chaytiep
	count = count + 1

def DUA_THUOC():
	global count
	global VT_hientai
	global sound_client
	global jobFinished
	global jobStart
	global pose_ht
	global complete
	if count ==0:
		sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/xinmoi_laythuoc.wav')
	if (count > 200) and (complete==1): #job is finished, chuyen tiep den cong viec tiep theo
		count = 0
		jobFinished = 1
		jobStart = 0
		pose_ht.orientation.x=1		# DA HOAN THANH NV
		VT_hientai.NV = 0 	# Da lam xong Task
		complete = 0
		sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/xincamon_chucmanhkhoe.wav')
		print "Thuc hien XONG NV"
		print "Job:", jobFinished,"DK:", VT_hientai.DK, "Run:", chaytiep
	count = count + 1

def THAM_KHAM():
	global count
	global VT_hientai
	global sound_client
	global jobFinished
	global jobStart
	global pose_ht
	global complete
	if count ==0:
		sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/xinmoibenhnhanranoichuyen.wav')
	if (count > 200) and (complete==1): #job is finished, chuyen tiep den cong viec tiep theo
		count = 0
		jobFinished = 1
		jobStart = 0
		pose_ht.orientation.x=1		# DA HOAN THANH NV
		VT_hientai.NV = 0 	# Da lam xong Task
		complete = 0
		sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/xincamon_chucvuive.wav')
		print "Thuc hien XONG NV"
		print "Job:", jobFinished,"DK:", VT_hientai.DK, "Run:", chaytiep
	count = count + 1

def LAY_RAC():
	global count
	global VT_hientai
	global sound_client
	global jobFinished
	global jobStart
	global pose_ht
	global complete
	if count ==0:
		sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/xinmoi_borac.wav')
	if (count > 200) and (complete==1): #job is finished, chuyen tiep den cong viec tiep theo
		count = 0
		jobFinished = 1
		jobStart = 0
		pose_ht.orientation.x=1		# DA HOAN THANH NV
		VT_hientai.NV = 0 	# Da lam xong Task
		complete = 0
		sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/xincamon_tambiet.wav')
		print "Thuc hien XONG NV"
		print "Job:", jobFinished,"DK:", VT_hientai.DK, "Run:", chaytiep
	count = count + 1

def Tranh_VC():
	global TaskList
	global jobFinished
	global VT_hientai
	global Laser
	global Vslow2
	global CB
	global homeID
	global chaytiep
	global vatcan
	global V
	global W
	global tranhduong
	global sound_client
	global PLC
	global addr_Front_Obs
	global addr_Back_Obs
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	global Front_Obs
	global Back_Obs

	# chuyen sang quan ly che do den, vat can

	if (Front_Obs==1):
		CB = True
	else:
		CB = False
	#=========== GUI LENH TOC DO DAT CHO DRIVER DK DONG CO ===============#
	if (len(TaskList)>0):
		if (jobFinished==1) and (VT_hientai.DK==0): # dang di chuyen
			if CB:
				if ((TaskList[0].ID==Sac) or (TaskList[0].ID==XuatPhat) or (TaskList[0].ID==LayXe)): # Tai nhung diem khong gian hep, tat che do vatcan
					chaytiep = 1
			else:
				chaytiep = 0
				vatcan = True
				V = 0
				W = 0
				if (tranhduong == 0):
					sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/denghitranhduong.wav')
					tranhduong=1
				else:
					vatcan = False
					chaytiep = 1
					tranhduong = 0

def Bam_VachTu():
	global dm
	global V
	global W
	global Vfast
	global Vslow2
	global dI
	global KP
	global KI
	global Wmax
	global dImax
	#---- Cap nhat V ------#
	if (abs(dm)<3):
		V= V+0.01
		if V > Vfast:
			V = Vfast
		dI=0
	else:
		V = V-0.005
		if V<Vslow2:
			V = Vslow2
			#dI = 0
	#---- Cap nhat W ------#
	dI = dI+0.02*dm
	if dI>dImax:
		dI = dImax
	elif dI<-dImax:
		dI = -dImax
		W = KP*dm+KI*dI
	if W > Wmax:
		W = Wmax
	elif W < -Wmax:
		W = -Wmax

def Quay180():
	global chaytiep
	global ck_turn
	global pre_lenc
	global pre_renc
	global lenc
	global renc
	global ngoai_vachtu
	global dm
	global VT_hientai
	global V
	global W
	global dI
	global Wmax
	global quaylai_ok
	global ck_quay
	#============== Quay 180 do  ======================#
	chaytiep=1
	if ck_turn == 0:
		ck_turn=1
		pre_lenc = lenc
		pre_renc = renc
		V = 0
		W = 0
		dI = 0
	if ((abs(pre_lenc-lenc)>260000) or (abs(pre_renc-renc)>260000)) and (ngoai_vachtu==0) and (abs(dm)<1):
		ck_turn=0	# Reset lai bien kiem tra viec quay xe
		VT_hientai.DK=0 # Het lenh quay/re

def DungXe():
	global chaytiep
	global TaskList
	global gotTask
	global started
	global VT_hientai
	global pose_ht
	global V
	global W
	global dI
	global NV
	global sound_client
	chaytiep = 0
	if len(TaskList)==1:
		sound_client.playWave("/opt/ros/melodic/share/sound_play/sounds/hoan_thanh_nhiem_vu.wav")
	#del TaskList[0]	# Xoa Task cuoi cung
	#gotTask=0		# Bao het NV
	#started=0		# Bao san sang nhan hanh trinh moi
	VT_hientai.ID = 0	# Reset lai Task
	INIT_JOB()	
	V = 0
	W = 0
	#dI = 0
	#NV = 0

def TiepTucDi():
	global chaytiep
	global TaskList
	global VT_hientai
	global V
	if VT_hientai.ID>0:
		chaytiep=1
		if V<0.05:
			V = 0.05
		del TaskList[0]
		VT_hientai.ID = 0
		
def getNgoaiVachtu(msg):
	global ngoai_vachtu
	ngoai_vachtu = msg.data
	

def ReTrai():
	global chaytiep
	global ck_turn
	global pre_lenc
	global pre_renc
	global lenc
	global renc
	global ngoai_vachtu
	global dm
	global VT_hientai
	global V
	global W
	global dI
	global Wmax
	global ck_turn
	chaytiep=1
	if ck_turn==0:
		ck_turn=1
		pre_lenc = lenc
		pre_renc = renc
		V = 0
		W = 0
		dI = 0
	if ((abs(pre_lenc-lenc)>35000) or (abs(pre_renc-renc)>35000)) and (ngoai_vachtu==0) and (abs(dm)<3):  # quay 40 do
		ck_turn=0	# Reset lai bien kiem tra viec quay xe
		VT_hientai.DK=0 # Het lenh quay/r

def RePhai():
	global chaytiep
	global ck_turn
	global pre_lenc
	global pre_renc
	global lenc
	global renc
	global dm
	global VT_hientai
	global V
	global W
	global Wmax
	chaytiep=1
	if ck_turn==0:
		ck_turn=1
		pre_lenc = lenc
		pre_renc = renc
		V = 0
		W = 0
		dI = 0
	if ((abs(pre_lenc-lenc)>35000) or (abs(pre_renc-renc)>35000)) and (ngoai_vachtu==0) and (abs(dm)<3):  # quay
		ck_turn=0	# Reset lai bien kiem tra viec quay xe
		VT_hientai.DK=0 # Het lenh quay/re

def LuiXe():
	global chaytiep
	global ck_lui
	global pre_lenc
	global pre_renc
	global lenc
	global renc
	global V
	global Vslow2
	global W
	global dI
	global VT_hientai
	global lui_tiep
	global cnt_lui
	global Back_Obs
	chaytiep=1
	if ck_lui==0:
		print "LuiXe"
		ck_lui=1
		pre_lenc = lenc
		pre_renc = renc
		V = 0
		W = 0
		dI = 0
		cnt_lui = 0
		lui_tiep=0
	#print abs(pre_lenc-lenc), abs(pre_renc-renc)
	if lui_tiep == 0:
		if ((abs(pre_lenc-lenc)>50000) and (abs(pre_renc-renc)>50000)):
			lui_tiep = 1
			chaytiep = 0
			ck_lui = 2	
  	elif lui_tiep==1:
		#print cnt_lui
		cnt_lui = cnt_lui+1
		if (cnt_lui > 150):
			lui_tiep = 2			
			ck_lui = 1	
	elif lui_tiep ==2:
		if ((abs(pre_lenc-lenc)>600000) and (abs(pre_renc-renc)>600000)):
			print "Lui xe xong"
			ck_lui=0	# Reset lai bien kiem tra viec quay xe
			VT_hientai.DK=3 # Het lenh lui
			V = 0
			W = 0
			lui_tiep = 0
			cnt_lui = 0

def LuiXe_Dung():
	global chaytiep
	global ck_lui
	global pre_lenc
	global pre_renc
	global lenc
	global renc
	global V
	global Vslow2
	global W
	global dI
	global VT_hientai
	chaytiep=1
	if ck_lui==0:
		ck_lui=1
		pre_lenc = lenc
		pre_renc = renc
		V = 0
		W = 0
		dI = 0
	if (abs(pre_lenc-lenc)>100000 and abs(pre_renc-renc)>100000):
		#print "Lui xe xong"
		ck_lui=0	# Reset lai bien kiem tra viec quay xe
		VT_hientai.DK=0 # Het lenh lui
		V = 0
		W = 0
		dI = 0
		chaytiep=0

def GiamToc():
	global TaskList
	global V
	global Vslow1
	global Vslow2
	global Vslow3
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	if(len(TaskList)>0):
		if (TaskList[0].ID == Sac):
			if (TaskList[0].DIR==-1): # ve tram sac
				V = V - 0.05
			if V < Vslow3:
				V = Vslow3
			else:    # Di khoi tram sac
				V = V - 0.02
			if V < Vslow2:
				V = Vslow2
		elif (TaskList[0].ID == LayRac) or (TaskList[0].ID == XuatPhat) or (TaskList[0].ID==PHONG[14]):
			V = V - 0.02
			if V < Vslow2:
				V = Vslow2
		elif (TaskList[0].ID==Cua2) or (TaskList[0].ID==LayXe):
			# V = Vslow3
			V = V - 0.02
			if V < Vslow2:
				V = Vslow2
		elif ((TaskList[0].ID==Cua1) and (TaskList[0].DIR == -1)): # chieu ve qua doan cua 1
			V = V - 0.02
			if V < Vslow2:
				V = Vslow2
		elif (TaskList[0].ID==PHONG[1]) and (TaskList[0].DIR==1):
			V = V - 0.02
			if V < Vslow2:
				V = Vslow2
		elif (TaskList[0].ID==PHONG[13]) and (TaskList[0].ID==-1):
			V = V - 0.02
			if V < Vslow2:
				V = Vslow2 

def INIT_JOB():
	global gotTask, TaskList, auto_on
	del TaskList[:]
	auto_on = 1  # thay cho mode == 0
	gotTask = 0
	print "RESET TASK"

def INIT():
	global PLC_rw_cnt
	global dx
	global dr
	global mode
	global dm
	global chaytiep
	global VT_hientai
	global nVT
	global ser
	global lenc
	global renc
	global L
	global R
	global idle
	global TaskList
	global jobFinished
	global jobStart
	global cuaStatus
	global DIR
	global turn
	global CB
	global Laser
	global Laser0
	global vatcan
	global gotTask
	global sound_client
	global VT_hientai
	global lui
	global ngoai_vachtu
	global LastID
	global homeID
	global pub_pose
	global pose_ht
	global started
	global count
	global NV_List
	global passed_rfid
	global V
	global W
	global tranhduong
	global light
	global complete
	global sigma
	global da_layXe
	global layxeID
	global load_xe
	global pub_dir
	global is_mangxe
	global N_ID
	global VT_Sac
	global Vfast
	global Vslow1
	global Vslow2
	global Vslow3
	global dI
	global KP
	global KI
	global dImax
	global Wmax
	global PLC
	global ck_turn
	global ck_lui
	global Front_Obs
	global Back_Obs

	global cnt
	global LIFT_UP
	global LIFT_DOWN
	global ck
	global ck_connection
	global vibot_state
	global quaylai_ok
	global ck_quay
	global is_reset
	global cmd_reset_cam
	global PC_treo
	global old_msg
	global auto_on
	global cnt_bamvachtu	
	global Bumper_Front
	global Bumper_Back
	global Estop
	global bumper_front_reset
	global bumper_back_reset
	global BAT	
	#========== THAM SO GUI TU SERVER ======================================#
	TaskList = [] 	# Luu hanh trinh di chuyen cua robot
	BAT = BatteryState()
	old_msg = -1
	auto_on = 1
	cnt_bamvachtu = 0
	Bumper_Front = 0
	Bumper_Back = 0
	Estop = 0
	bumper_front_reset = 0
	bumper_back_reset = 0
	vibot_state = [0,0,0,0,0]
	ck_quay = 1
	PC_treo = 0
	cmd_reset_cam = 0
	LIFT_UP=0
	is_reset = 0
	LIFT_DOWN=0
	quaylai_ok=1
	ck =0
	ck_connection = 0
	cnt = 0
	load_xe = 1
	table_pos = 0
	Front_Obs = 0
	Back_Obs = 0
	N_ID = 0
	is_mangxe = 0
	passed_rfid = 0
	gotTask = 0
	da_layXe = 0
	jobFinished = 1
	cuaStatus = 0
	jobStart=0
	passed_node = 1
	next_node=1
	tranhduong=0
	complete = 0
	Laser = 5
	Laser0 = 0.0
	VT_hientai = Node(0,0,0,0)# Luu vi tri tai diem nut hien tai cua robot
	pose_ht = Pose()
	pose_ht.orientation.x = 0
	started = 0		# Kiem tra HT bat dau chay hay chua

		#========== THAM SO CHUNG HE THONG =====================================#
	mode = 0 	# dk bang tay
	L = 0.223 	# Khoang cach giua hai banh
	R = 0.075 	# Ban kinh banh xe
	chaytiep = 1	# flag cho phep dong co hoat dong hay dung
	left = 0.0   	# van toc dat cho banh trai
	right = 0.0	# van toc dat cho banh phai
	lenc = 0  	# encoder trai
	renc = 0  	# encoder phai
	V = 0.0  	# van toc dai desired
	W = 0.0  	# van toc goc desired
	dx = 0.0	# van toc dai lay tu joystick
	dr = 0.0	# van toc goc lay tu Joystick
	dm = 0.0  	# sai lech so voi vach tu
	turn = 555	# Quay
	DIR = 0		# Huong quay cua robot: 1: Quay phai, -1: Quay trai
	ck_turn = 0	# Kiem tra xem da thuc hien xong viec quay xe chua
	ck_lui = 0
	vatcan = 0 	# Kiem tra xem co vat can hay khong
	CB=False
	#========= THAM SO DK BAM VACH TU ======================================#
	dI = 0	 	# Dai luong Tich phan
	KP = 0.04	# He so ti le
	KI = 0.07
	dImax = 3
	Vfast = 0.4 	# Van toc di chuyen theo duong thang m/s
	Vslow1= 0.15	# Van toc di chuyen theo duong cong (doan re)
	Vslow2 = 0.1
	Vslow3 = 0.05
	Wmax = 0.5	# Van toc quay xe lon nhat (rad/s)
	tick = 0 	# Bien quan ly thoi gian di vao khuc cua
	cua = 0 	# Bien kiem tra xem dang o doan cua hay ko
	ngoai_vachtu=0	# kiem tra xe cos bam vach tu hay khong

def CT_DuaCom_14():
	global TaskList
	global started
	global gotTask
	global chaytiep
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	del TaskList[:]
	TaskList.append(Node(PHONG[12], 0, 0,1))	# P12
	TaskList.append(Node(PHONG[13], 0, 0,1))	# P13
	TaskList.append(Node(Cua2, 1, 0,1))	# Nga ba lay do
	TaskList.append(Node(LayXe, 3, 5,1))	# Diem Lay xe com
	# Chay dua com den tung phong
	TaskList.append(Node(Cua2, 1, 0,-1))	# Nga ba lay do
	TaskList.append(Node(PHONG[14], 3, 1,-1))	# P14
	TaskList.append(Node(Cua2, 0, 0,-1))	# Nga ba lay do
	TaskList.append(Node(PHONG[13], 0, 1,-1))	# P13
	TaskList.append(Node(PHONG[12], 4, 1,-1))	# P12
	started =1
	gotTask = 1
	chaytiep=1
	N = len(TaskList)
	for i in range(0,N):
		print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV

def CT_LayRac_14():
	global TaskList
	global started
	global gotTask
	global chaytiep
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG

	del TaskList[:]
	#TaskList.append(Node(PHONG[12], 0, 0,1))	# P12
	#TaskList.append(Node(PHONG[13], 0, 0,1))	# P13
	TaskList.append(Node(Cua2, 0, 0,1))	# Nga ba lay do
	# Chay dua com den tung phong
	TaskList.append(Node(PHONG[14], 3, 4,-1))	# P14
	TaskList.append(Node(Cua2, 0, 0,-1))	# Nga ba lay do
	TaskList.append(Node(PHONG[13], 0, 4,-1))	# P13
	TaskList.append(Node(PHONG[12], 4, 4,-1))	# P12

	started =1
	gotTask = 1
	chaytiep=1
	N = len(TaskList)
	for i in range(0,N):
		print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV


def CT_DuaCom_SHORT():
	global TaskList
	global started
	global gotTask
	global chaytiep
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	del TaskList[:]

	# Chay dua com den tung phong
	TaskList.append(Node(PHONG[13], 0, 1,-1))	# P13
	TaskList.append(Node(PHONG[12], 0, 1,-1))	# P12
	TaskList.append(Node(PHONG[11], 0, 1,-1))	# P11
	TaskList.append(Node(PHONG[10], 0, 1,-1))	# P10
	TaskList.append(Node(PHONG[9], 0, 1,-1))	# P9
	TaskList.append(Node(PHONG[8], 0, 1,-1))	# P8
	TaskList.append(Node(PHONG[7], 0, 1,-1))	# P7
	TaskList.append(Node(PHONG[6], 0, 1,-1))	# P6
	TaskList.append(Node(PHONG[5], 0, 1,-1))	# P5
	TaskList.append(Node(PHONG[4], 0, 1,-1))	# P4
	TaskList.append(Node(PHONG[3], 0, 1,-1))	# P3
	TaskList.append(Node(PHONG[2], 0, 1,-1))	# P2
	TaskList.append(Node(PHONG[1], 0, 1,-1))	# P1
	TaskList.append(Node(Cua1, 0, 0,-1))	# Diem cua
	TaskList.append(Node(LayRac, 0, 0,-1))	# Diem Lay Rac
	TaskList.append(Node(XuatPhat, 0, 6,-1))	# Diem XP - Home - Tra xe com
	TaskList.append(Node(Sac, 4, 0,-1))	# VT sac pin
	# Bat lai cac flag kiem tra

	#TaskList.append(Node(PHONG[13], 0, 6,-1))	# P13
	#TaskList.append(Node(PHONG[12], 5, 0,-1))	# P12
	started =1
	gotTask = 1
	chaytiep=1
	N = len(TaskList)
	for i in range(0,N):
		print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV

def CT_DuaCom_FULL():
	global TaskList
	global started
	global gotTask
	global chaytiep
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	global NV
	NV = 1
	del TaskList[:]

	# Chay tu Gara >> Diem lay com
	TaskList.append(Node(Sac, 9, 0,1))		# VT sac pin
	TaskList.append(Node(XuatPhat, 0, 0,1))	# Diem XP - Home
	TaskList.append(Node(LayRac, 0, 0,1))	# Diem Lay Rac
	TaskList.append(Node(Cua1, 0, 0,1))	# Diem cua
	TaskList.append(Node(PHONG[1], 0, 0,1))	# P1
	TaskList.append(Node(PHONG[2], 0, 0,1))	# P2
	TaskList.append(Node(PHONG[3], 0, 0,1))	# P3
	TaskList.append(Node(PHONG[4], 0, 0,1))	# P4
	TaskList.append(Node(PHONG[5], 0, 0,1))	# P5
	TaskList.append(Node(PHONG[6], 0, 0,1))	# P6
	TaskList.append(Node(PHONG[7], 0, 0,1))	# P7
	TaskList.append(Node(PHONG[8], 0, 0,1))	# P8
	TaskList.append(Node(PHONG[9], 0, 0,1))	# P9
	TaskList.append(Node(PHONG[10], 0, 0,1))	# P10
	TaskList.append(Node(PHONG[11], 0, 0,1))	# P11
	TaskList.append(Node(PHONG[12], 0, 0,1))	# P12
	TaskList.append(Node(PHONG[13], 0, 0,1))	# P13
	TaskList.append(Node(Cua2, 1, 0,1))	# Nga ba lay do
	TaskList.append(Node(LayXe, 3, 5,1))	# Diem Lay xe com
	# Chay dua com den tung phong
	TaskList.append(Node(Cua2, 1, 0,-1))	# Nga ba lay do
	TaskList.append(Node(PHONG[14], 3, 1,-1))	# P14
	TaskList.append(Node(PHONG[13], 0, 0,-1))	# P13
	TaskList.append(Node(PHONG[12], 0, 0,-1))	# P12
	TaskList.append(Node(PHONG[11], 0, 1,-1))	# P11
	TaskList.append(Node(PHONG[10], 0, 0,-1))	# P10
	TaskList.append(Node(PHONG[9], 0, 1,-1))	# P9
	TaskList.append(Node(PHONG[8], 0, 1,-1))	# P8
	TaskList.append(Node(PHONG[7], 0, 1,-1))	# P7
	TaskList.append(Node(PHONG[6], 0, 0,-1))	# P6
	TaskList.append(Node(PHONG[5], 0, 1,-1))	# P5
	TaskList.append(Node(PHONG[4], 0, 0,-1))	# P4
	TaskList.append(Node(PHONG[3], 0, 1,-1))	# P3
	TaskList.append(Node(PHONG[2], 3, 1,-1))	# P2
	# Quay lai tra xe
	TaskList.append(Node(PHONG[3], 0, 0, 1))  # P3
	TaskList.append(Node(PHONG[4], 0, 0, 1))  # P4
	TaskList.append(Node(PHONG[5], 0, 0, 1))  # P5
	TaskList.append(Node(PHONG[6], 0, 0, 1))  # P6
	TaskList.append(Node(PHONG[7], 0, 0, 1))  # P7
	TaskList.append(Node(PHONG[8], 0, 0, 1))  # P8
	TaskList.append(Node(PHONG[9], 0, 0, 1))  # P9
	TaskList.append(Node(PHONG[10], 0, 0, 1))  # P10
	TaskList.append(Node(PHONG[11], 0, 0, 1))  # P11
	TaskList.append(Node(PHONG[12], 0, 0, 1))  # P12
	TaskList.append(Node(PHONG[13], 0, 0, 1))  # P13
	TaskList.append(Node(Cua2, 1, 0, 1))  # Nga ba lay do
	TaskList.append(Node(LayXe, 3, 6, 1))  # Diem tra xe com
	# Quay lai tram sac
	TaskList.append(Node(Cua2, 2, 0, -1))  # Nga ba lay do
	TaskList.append(Node(PHONG[13], 0, 0, -1))  # P13
	TaskList.append(Node(PHONG[12], 0, 0, -1))  # P12
	TaskList.append(Node(PHONG[11], 0, 0, -1))  # P11
	TaskList.append(Node(PHONG[10], 0, 0, -1))  # P10
	TaskList.append(Node(PHONG[9], 0, 0, -1))  # P9
	TaskList.append(Node(PHONG[8], 0, 0, -1))  # P8
	TaskList.append(Node(PHONG[7], 0, 0, -1))  # P7
	TaskList.append(Node(PHONG[6], 0, 0, -1))  # P6
	TaskList.append(Node(PHONG[5], 0, 0, -1))  # P5
	TaskList.append(Node(PHONG[4], 0, 0, -1))  # P4
	TaskList.append(Node(PHONG[3], 0, 0, -1))  # P3
	TaskList.append(Node(PHONG[2], 0, 0, -1))  # P2
	TaskList.append(Node(PHONG[1], 0, 0,-1))	# P1
	TaskList.append(Node(Cua1, 0, 0,-1))	# Diem cua
	TaskList.append(Node(LayRac, 0, 0,-1))	# Diem Lay Rac
	TaskList.append(Node(XuatPhat, 0, 0,-1))	# Diem XP - Home - Tra xe com
	TaskList.append(Node(Sac, 4, 0,-1))	# VT sac pin
	# Bat lai cac flag kiem tra
	
	gotTask = 1
	chaytiep=1
	N = len(TaskList)
	for i in range(0,N):
		print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV

def CT_DuaThuoc_FULL():
	global TaskList
	global started
	global gotTask
	global chaytiep
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	global NV
	NV = 2
	del TaskList[:]
	# Chay tu Gara >> Diem lay thuoc
	TaskList.append(Node(Sac, 9, 0,1))		# VT sac pin
	TaskList.append(Node(XuatPhat, 0, 0,1))	# Diem XP - Home
	TaskList.append(Node(LayRac, 0, 0,1))	# Diem Lay Rac
	TaskList.append(Node(Cua1, 0, 0,1))	# Diem cua
	TaskList.append(Node(PHONG[1], 0, 0,1))	# P1
	TaskList.append(Node(PHONG[2], 0, 0,1))	# P2
	TaskList.append(Node(PHONG[3], 0, 0,1))	# P3
	TaskList.append(Node(PHONG[4], 0, 0,1))	# P4
	TaskList.append(Node(PHONG[5], 0, 0,1))	# P5
	TaskList.append(Node(PHONG[6], 0, 0,1))	# P6
	TaskList.append(Node(PHONG[7], 0, 0,1))	# P7
	TaskList.append(Node(PHONG[8], 0, 0,1))	# P8
	TaskList.append(Node(PHONG[9], 0, 0,1))	# P9
	TaskList.append(Node(PHONG[10], 0, 0,1))	# P10
	TaskList.append(Node(PHONG[11], 0, 0,1))	# P11
	TaskList.append(Node(PHONG[12], 0, 0,1))	# P12
	TaskList.append(Node(PHONG[13], 0, 0,1))	# P13
	TaskList.append(Node(Cua2, 1, 0,1))	# Nga ba lay do
	TaskList.append(Node(LayXe, 3, 5,1))	# Diem Lay xe thuoc
	# Chay dua thuoc den tung phong
	TaskList.append(Node(Cua2, 1, 0,-1))	# Nga ba lay do
	TaskList.append(Node(PHONG[14], 3, 2,-1))	# P14
	TaskList.append(Node(PHONG[13], 0, 0,-1))	# P13
	TaskList.append(Node(PHONG[12], 0, 0,-1))	# P12
	TaskList.append(Node(PHONG[11], 0, 2,-1))	# P11
	TaskList.append(Node(PHONG[10], 0, 0,-1))	# P10
	TaskList.append(Node(PHONG[9], 0, 2,-1))	# P9
	TaskList.append(Node(PHONG[8], 0, 2,-1))	# P8
	TaskList.append(Node(PHONG[7], 0, 2,-1))	# P7
	TaskList.append(Node(PHONG[6], 0, 0,-1))	# P6
	TaskList.append(Node(PHONG[5], 0, 2,-1))	# P5
	TaskList.append(Node(PHONG[4], 0, 0,-1))	# P4
	TaskList.append(Node(PHONG[3], 0, 2,-1))	# P3
	TaskList.append(Node(PHONG[2], 3, 2,-1))	# P2
	# Quay lai tra xe
	TaskList.append(Node(PHONG[3], 0, 0, 1))  # P3
	TaskList.append(Node(PHONG[4], 0, 0, 1))  # P4
	TaskList.append(Node(PHONG[5], 0, 0, 1))  # P5
	TaskList.append(Node(PHONG[6], 0, 0, 1))  # P6
	TaskList.append(Node(PHONG[7], 0, 0, 1))  # P7
	TaskList.append(Node(PHONG[8], 0, 0, 1))  # P8
	TaskList.append(Node(PHONG[9], 0, 0, 1))  # P9
	TaskList.append(Node(PHONG[10], 0, 0, 1))  # P10
	TaskList.append(Node(PHONG[11], 0, 0, 1))  # P11
	TaskList.append(Node(PHONG[12], 0, 0, 1))  # P12
	TaskList.append(Node(PHONG[13], 0, 0, 1))  # P13
	TaskList.append(Node(Cua2, 1, 0, 1))  # Nga ba lay do
	TaskList.append(Node(LayXe, 3, 6, 1))  # Diem tra xe thuoc
	# Quay lai tram sac
	TaskList.append(Node(Cua2, 2, 0, -1))  # Nga ba lay do
	TaskList.append(Node(PHONG[13], 0, 0, -1))  # P13
	TaskList.append(Node(PHONG[12], 0, 0, -1))  # P12
	TaskList.append(Node(PHONG[11], 0, 0, -1))  # P11
	TaskList.append(Node(PHONG[10], 0, 0, -1))  # P10
	TaskList.append(Node(PHONG[9], 0, 0, -1))  # P9
	TaskList.append(Node(PHONG[8], 0, 0, -1))  # P8
	TaskList.append(Node(PHONG[7], 0, 0, -1))  # P7
	TaskList.append(Node(PHONG[6], 0, 0, -1))  # P6
	TaskList.append(Node(PHONG[5], 0, 0, -1))  # P5
	TaskList.append(Node(PHONG[4], 0, 0, -1))  # P4
	TaskList.append(Node(PHONG[3], 0, 0, -1))  # P3
	TaskList.append(Node(PHONG[2], 0, 0, -1))  # P2
	TaskList.append(Node(PHONG[1], 0, 0, -1))  # P1
	TaskList.append(Node(Cua1, 0, 0, -1))  # Diem cua
	TaskList.append(Node(LayRac, 0, 0, -1))  # Diem Lay Rac
	TaskList.append(Node(XuatPhat, 0, 0, -1))  # Diem XP - Home - Tra xe com
	TaskList.append(Node(Sac, 4, 0,-1))	# VT sac pin
	# Bat lai cac flag kiem tra
	started =1
	gotTask = 1
	chaytiep=1
	N = len(TaskList)
	for i in range(0,N):
		print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV

def CT_LayRac_FULL():
	global TaskList
	global started
	global gotTask
	global chaytiep
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	global NV
	NV = 4
	del TaskList[:]
	# Chay tu Gara >> Diem lay com
	TaskList.append(Node(Sac, 9, 0,1))		# VT sac pin
	TaskList.append(Node(XuatPhat, 0, 0,1))	# Diem XP - Home
	TaskList.append(Node(LayRac, 0, 5,1))	# Diem Lay Rac
	TaskList.append(Node(Cua1, 0, 0,1))	# Diem cua
	TaskList.append(Node(PHONG[1], 0, 0,1))	# P1
	TaskList.append(Node(PHONG[2], 0, 0,1))	# P2
	TaskList.append(Node(PHONG[3], 0, 0,1))	# P3
	TaskList.append(Node(PHONG[4], 0, 0,1))	# P4
	TaskList.append(Node(PHONG[5], 0, 0,1))	# P5
	TaskList.append(Node(PHONG[6], 0, 0,1))	# P6
	TaskList.append(Node(PHONG[7], 0, 0,1))	# P7
	TaskList.append(Node(PHONG[8], 0, 0,1))	# P8
	TaskList.append(Node(PHONG[9], 0, 0,1))	# P9
	TaskList.append(Node(PHONG[10], 0, 0,1))	# P10
	TaskList.append(Node(PHONG[11], 0, 0,1))	# P11
	TaskList.append(Node(PHONG[12], 0, 0,1))	# P12
	TaskList.append(Node(PHONG[13], 0, 0,1))	# P13
	TaskList.append(Node(Cua2, 0, 0,1))	# Diem cua 2
	# Chay dua com den tung phong
	TaskList.append(Node(PHONG[14], 3, 4,-1))	# P14
	TaskList.append(Node(PHONG[13], 0, 0,-1))	# P13
	TaskList.append(Node(PHONG[12], 0, 0,-1))	# P12
	TaskList.append(Node(PHONG[11], 0, 4,-1))	# P11
	TaskList.append(Node(PHONG[10], 0, 0,-1))	# P10
	TaskList.append(Node(PHONG[9], 0, 4,-1))	# P9
	TaskList.append(Node(PHONG[8], 0, 4,-1))	# P8
	TaskList.append(Node(PHONG[7], 0, 4,-1))	# P7
	TaskList.append(Node(PHONG[6], 0, 0,-1))	# P6
	TaskList.append(Node(PHONG[5], 0, 4,-1))	# P5
	TaskList.append(Node(PHONG[4], 0, 0,-1))	# P4
	TaskList.append(Node(PHONG[3], 0, 4,-1))	# P3
	TaskList.append(Node(PHONG[2], 0, 4,-1))	# P2
	TaskList.append(Node(PHONG[1], 0, 0,-1))	# P1
	TaskList.append(Node(Cua1, 0, 0,-1))	# Diem cua
	TaskList.append(Node(LayRac, 0, 6,-1))	# Diem Lay Rac
	TaskList.append(Node(XuatPhat, 0, 0,-1))	# Diem XP - Home 
	TaskList.append(Node(Sac, 4, 0,-1))	# VT sac pin
	# Bat lai cac flag kiem tra
	started =1
	gotTask = 1
	chaytiep=1
	N = len(TaskList)
	for i in range(0,N):
		print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV


def CT_ThamKham_FULL():
	global TaskList
	global started
	global gotTask
	global chaytiep
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	global NV
	NV = 3
	del TaskList[:]
	# Chay tu Gara >> Diem lay com
	TaskList.append(Node(Sac, 9, 0,1))		# VT sac pin
	TaskList.append(Node(XuatPhat, 0, 0,1))	# Diem XP - Home
	TaskList.append(Node(LayRac, 0, 0,1))	# Diem Lay Rac
	TaskList.append(Node(Cua1, 0, 0,1))	# Diem cua
	TaskList.append(Node(PHONG[1], 0, 0,1))	# P1
	TaskList.append(Node(PHONG[2], 0, 2,1))	# P2
	TaskList.append(Node(PHONG[3], 0, 3,1))	# P3
	TaskList.append(Node(PHONG[4], 0, 0,1))	# P4
	TaskList.append(Node(PHONG[5], 0, 3,1))	# P5
	TaskList.append(Node(PHONG[6], 0, 0,1))	# P6
	TaskList.append(Node(PHONG[7], 0, 3,1))	# P7
	TaskList.append(Node(PHONG[8], 0, 3,1))	# P8
	TaskList.append(Node(PHONG[9], 0, 3,1))	# P9
	TaskList.append(Node(PHONG[10], 0, 0,1))	# P10
	TaskList.append(Node(PHONG[11], 0, 3,1))	# P11
	TaskList.append(Node(PHONG[12], 0, 0,1))	# P12
	TaskList.append(Node(PHONG[13], 0, 0,1))	# P13
	TaskList.append(Node(Cua2, 0, 0,1))	# Diem cua 2

	TaskList.append(Node(PHONG[14], 3, 3,-1))	# P14
	TaskList.append(Node(PHONG[13], 0, 0,-1))	# P13
	TaskList.append(Node(PHONG[12], 0, 0,-1))	# P12
	TaskList.append(Node(PHONG[11], 0, 0,-1))	# P11
	TaskList.append(Node(PHONG[10], 0, 0,-1))	# P10
	TaskList.append(Node(PHONG[9], 0, 0,-1))	# P9
	TaskList.append(Node(PHONG[8], 0, 0,-1))	# P8
	TaskList.append(Node(PHONG[7], 0, 0,-1))	# P7
	TaskList.append(Node(PHONG[6], 0, 0,-1))	# P6
	TaskList.append(Node(PHONG[5], 0, 0,-1))	# P5
	TaskList.append(Node(PHONG[4], 0, 0,-1))	# P4
	TaskList.append(Node(PHONG[3], 0, 0,-1))	# P3
	TaskList.append(Node(PHONG[2], 0, 0,-1))	# P2
	TaskList.append(Node(PHONG[1], 0, 0,-1))	# P1
	TaskList.append(Node(Cua1, 0, 0,-1))	# Diem cua
	TaskList.append(Node(LayRac, 0, 0,-1))	# Diem Lay Rac
	TaskList.append(Node(XuatPhat, 0, 0,-1))	# Diem XP - Home - Tra xe com
	TaskList.append(Node(Sac, 4, 0,-1))	# VT sac pin
	started =1
	gotTask = 1
	chaytiep=1
	N = len(TaskList)
	for i in range(0,N):
		print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV

# def GetCT(msg):
# 	global NV
# 	global mode
# 	global chaytiep
# 	global quaylai_ok
# 	#print msg.data, "GetCT, gotTask: ", gotTask, "run: ", chaytiep, "quaylai_ok:", quaylai_ok
#
# 	if (msg.data == 1) and (gotTask==0): 	# CT_DuaCom_FULL
# 	    CT_DuaCom_FULL()
# 	    NV = 1
# 	elif (msg.data == 2) and (gotTask==0): 	# CT_DuaThuoc_FULL
# 	    CT_DuaThuoc_FULL()
# 	    NV = 2
# 	elif (msg.data == 3) and (gotTask==0): 	# CT_ThamKham_FULL
# 	    CT_ThamKham_FULL()
# 	    NV = 3
# 	elif (msg.data == 4) and (gotTask==0): 	# CT_LayRac
# 	    CT_LayRac_FULL()
# 	    NV = 4
# 	elif (msg.data == 0) and ((gotTask==1) or (mode==1)): 	# Xoa tat ca cac CT hien tai hoac o che do dk bang tay (MODE == 1)
# 	    INIT_JOB()

def INIT_PATH_BENH_VIEN():
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	
	Sac = 3917728665	#4188503193
	XuatPhat = 153970584
	LayRac = 2302261656
	Cua1 = 2835506841
	Cua2 = 971138969
	LayXe= 4180957593	
	PHONG = [0,1499122585,1773960856,3373888153,2568145560,1767016856,691197592,3920326296,3386032281,2579414681,424529561,4193108633,3111635609,4183412121,698465433]
	FULL_PATH=[Sac,XuatPhat,LayRac,Cua1,PHONG[1],PHONG[2],PHONG[3],PHONG[4],PHONG[5],PHONG[6],PHONG[7],PHONG[8],PHONG[9],PHONG[10],PHONG[11],PHONG[12],PHONG[13],Cua2,LayXe, PHONG[14]]

def INIT_PATH_HOC_VIEN():
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	Sac = 699283609
	XuatPhat = 2033174169
	LayRac = 3110321049
	Cua1 = 2569456793
	Cua2 = 4177623960 #422478488
	LayXe= 419534232
	#P14 = 4177623960
	PHONG = [0,3916852888,1777320089,2303704729,1499708056,2300559513,3652089496,2843435672,1237023128,1240624536,2582758297,160745113,427125144,2837012889,969213839]
	FULL_PATH=[Sac,XuatPhat,LayRac,Cua1,PHONG[1],PHONG[2],PHONG[3],PHONG[4],PHONG[5],PHONG[6],PHONG[7],PHONG[8],PHONG[9],PHONG[10],PHONG[11],PHONG[12],PHONG[13],Cua2,LayXe, PHONG[14]]
	#print FULL_PATH


def getINDEX(ID):
	global FULL_PATH
	for i in range(0,len(FULL_PATH)):
		if FULL_PATH[i]==ID:
			return i
	return -1


def getCmdPath():
	global TaskList
	global gotTask
	global pose_ht
	global started
	global chaytiep
	global vibot_state
	global XuatPhat
	global NV
	global FULL_PATH
	global PHONG
	global VT_hientai
	global old_msg
	#FULL_PATH=[Sac,XuatPhat,LayRac,Cua1,PHONG[1:13],Cua2,LayXe]
	#print "getCmdPath"
	if gotTask == 0:
		for i in range(0,len(msg.poses)):
			TaskList.append(Node(msg.poses[i].position.x,msg.poses[i].position.y,msg.poses[i].position.z,0))
		N = len(TaskList)
		print "---------------------------------------------------"
		for i in range(0,N):
			print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV, TaskList[i].DIR
		if (TaskList[i].NV < 5) and (TaskList[i].NV>0):
			NV = TaskList[i].NV
		gotTask = 1
		if len(TaskList)>1: # co it nhat 2 diem tren quy dao, tinh ca diem robot dang dung
			chieu = sign(getINDEX(TaskList[1].ID)-getINDEX(TaskList[0].ID))
			print "chieu:", chieu, started
		else:
			chieu = 0
			print "chieu:", chieu
		if (chieu!=0):
		# XU LY CHIEU DI CUA ROBOT
			N = len(TaskList)
			for i in range(0,N):
				print getINDEX(TaskList[i].ID)-3
		   # print FULL_PATH[i]
			for i in range(0,N-1):
				TaskList[i].DIR = sign(getINDEX(TaskList[i+1].ID)-getINDEX(TaskList[i].ID))
				#print "ck:", i+1, getINDEX(TaskList[i+1].ID), getINDEX(TaskList[i].ID)
			TaskList[-1].DIR = TaskList[-2].DIR
			pose_ht.orientation.x = 0

			print "TASK LIST"
			N = len(TaskList)
			for i in range(0,N):
				print getINDEX(TaskList[i].ID)-3, TaskList[i].DK, TaskList[i].NV, TaskList[i].DIR

			print "NEW TASK: ID: ", VT_hientai.ID, "NV: ", VT_hientai.NV, "DK: ", VT_hientai.DK
			chaytiep = 1
			V = 0
			W = 0
			dI = 0
			VT_hientai.ID = TaskList[0].ID
			VT_hientai.NV = TaskList[0].NV
			VT_hientai.DK = TaskList[0].DK
			VT_hientai.DIR = TaskList[0].DIR
			#pose_ht.orientation.y = pose_ht.position.x  # Luu OLD ID
				#pose_ht.position.x = TaskList[0].ID	    # NEW ID
			pose_ht.position.y = TaskList[0].DK	    # NEW DK
			pose_ht.position.z = TaskList[0].NV	    # NEW NV
			print "NEW TASK: ID: ", getINDEX(VT_hientai.ID)-3, "NV: ", VT_hientai.NV, "DK: ", VT_hientai.DK
			old_msg = -1
		else:
			del TaskList[:]
			gotTask=0


def main_Move():
	global jobFinished
	global VT_hientai
	global homeID
	global chaytiep
	global vatcan
	global V
	global W
	global tranhduong
	global sound_client
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	global Front_Obs
	global Back_Obs
	global Bumper_Front
	global Bumper_Back

	global TaskList
	global Vslow1
	global Vslow2
	global Vslow3
	global FULL_PATH

	global dm
	global Vfast
	global dI
	global KP
	global KI
	global Wmax
	global dImax

	if (VT_hientai.NV== 0) and (VT_hientai.DK == 0):
		#print "TaskList:", TaskList[0].ID, Sac
		# ----- GIAM TOC ---------#
		if (TaskList[0].ID == Sac):
			if (TaskList[0].DIR == -1):  # ve tram sac
				V = V - 0.01
				if V < Vslow3:
					V = Vslow3
			else:  # Di khoi tram sac
				V = V - 0.01
				if V < Vslow2:
					V = Vslow2
		elif ((TaskList[0].ID == LayRac) and (TaskList[0].DIR==1)) or ((TaskList[0].ID == PHONG[13]) and (TaskList[0].DIR==-1)) or (TaskList[0].ID == PHONG[14]):
			V = V - 0.01
			if V < Vslow2:
				V = Vslow2
		elif (TaskList[0].ID == Cua2) or (TaskList[0].ID == LayXe):
			# V = Vslow3
			V = V - 0.01
			if V < Vslow2:
				V = Vslow2
		elif ((TaskList[0].ID == Cua1) and (TaskList[0].DIR == -1)):  # chieu ve qua doan cua 1
			V = V - 0.01
			if V < Vslow2:
				V = Vslow2
		elif (TaskList[0].ID == PHONG[1]) and (TaskList[0].DIR == 1):
			V = V - 0.01
			if V < Vslow2:
				V = Vslow2

		# ---- TRANH VAT CAN ----#
		#=============== CAM BIEN VA CHAM =====================#
		if (Bumper_Front==1):
			if TaskList[0].ID == Sac:
				chaytiep = 1
			else:
				chaytiep = 0
				vatcan = True
				V = 0
				W = 0
				if (tranhduong == 0):
					sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/denghitranhduong.wav')
					tranhduong = 1	
		
		#============== CAM BIEN HONG NGOAI ===================#
		elif (Front_Obs == 1):
			if (TaskList[0].ID == Sac) or (TaskList[0].ID == XuatPhat) or (TaskList[0].ID == LayXe) or (TaskList[0].ID == PHONG[1]) or ((TaskList[0].ID == Cua1) and (TaskList[0].DIR==-1)) or (TaskList[0].ID == Cua2) or ((TaskList[0].ID == PHONG[13]) and (TaskList[0].DIR==-1)) or (TaskList[0].ID==PHONG[14]):  # Tai nhung diem khong gian hep, tat che do vatcan
				chaytiep = 1
			else:
				chaytiep = 0
				vatcan = True
				V = 0
				W = 0
				if (tranhduong == 0):
					sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/denghitranhduong.wav')
					tranhduong = 1
		else:
			vatcan = False  #???
			chaytiep = 1			
			tranhduong = 0

		# ---- BAM VACH TU ------#
		if (abs(dm) < 3):
			V = V + 0.002
			if V > Vfast:
				V = Vfast
			dI = 0
		else:
			V = V - 0.002
			if V < Vslow2:
				V = Vslow2
		# dI = 0
		# ---- Cap nhat W ------#
		dI = dI + 0.02 * dm
		if dI > dImax:
			dI = dImax
		elif dI < -dImax:
			dI = -dImax				
		W = KP * dm + KI * dI
			
		if W > Wmax:
			W = Wmax
		elif W < -Wmax:
			W = -Wmax

def main_Move_BamVach():
	global jobFinished
	global VT_hientai
	global homeID
	global chaytiep
	global vatcan
	global V
	global W
	global tranhduong
	global sound_client
	global FULL_PATH
	global Sac
	global XuatPhat
	global LayRac
	global Cua1
	global Cua2
	global LayXe
	global PHONG
	global Front_Obs
	global Back_Obs
	global Bumper_Front
	global Bumper_Back

	global TaskList
	global Vslow1
	global Vslow2
	global Vslow3
	global FULL_PATH

	global dm
	global Vfast
	global dI
	global KP
	global KI
	global Wmax
	global dImax
	
	chaytiep = 1
	# ---- TRANH VAT CAN ----#
	if (Bumper_Front == 1):
		chaytiep = 0
		V = 0
		W = 0
		if (tranhduong == 0):
			sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/denghitranhduong.wav')
			tranhduong = 1	
	elif (Front_Obs == 1):
		V = Vslow2
		if (tranhduong == 0):
			sound_client.playWave('/opt/ros/melodic/share/sound_play/sounds/denghitranhduong.wav')
			tranhduong = 1
	else:
		V = Vslow2
		vatcan = False  #???
		chaytiep = 1
		tranhduong = 0

	# ---- BAM VACH TU ------#
	if (abs(dm) < 3):
		V = V + 0.002
		if V > Vfast:
			V = Vfast
		dI = 0
	else:
		V = V - 0.002
		if V < Vslow2:
			V = Vslow2
	# dI = 0
	# ---- Cap nhat W ------#
	dI = dI + 0.02 * dm
	if dI > dImax:
		dI = dImax
	elif dI < -dImax:
		dI = -dImax				
	W = KP * dm + KI * dI
		
	if W > Wmax:
		W = Wmax
	elif W < -Wmax:
		W = -Wmax
#==========================================================================#
#================================= MAIN_NV() =============================#
def main_NV():
	global VT_hientai, chaytiep, complete, V, W, dI, jobFinished, pose_ht
	global PHONG, count, jobStart

	if (VT_hientai.NV > 0):
		# if (VT_hientai.NV==5) or (VT_hientai.NV==6): # LAY/TRA Xe
		if (VT_hientai.NV == 5) or (VT_hientai.NV==6)  or (((VT_hientai.NV == 1) or (VT_hientai.NV == 2) or (VT_hientai.NV == 4)) and (VT_hientai.ID == PHONG[14])):
			if (VT_hientai.DK == 3):
				#print "180"
				Quay180()
			else:
				if chaytiep == 1:
					chaytiep = 0
					complete = 0
					V = 0
					W = 0
					dI = 0
					jobFinished = 0
					pose_ht.orientation.x = 0
					count = 0
					# print "Bat dau NV"
					jobStart = 1
				elif (jobFinished == 0):
					# jobProcess()
					jobProcess2()                 
		else:
			if chaytiep == 1:
				chaytiep = 0
				complete = 0
				V = 0
				W = 0
				dI = 0
				jobFinished = 0
				pose_ht.orientation.x = 0
				count = 0
				# print "Bat dau NV"
				jobStart = 1
			elif (jobFinished == 0):
				# print "Test job"
				jobProcess2()

#=========================================================================#
#============================= MAIN_DK() =================================#
def main_DK():
	global VT_hientai, chaytiep, V, W, ck_turn, ck_lui, Wmax, Vslow2
	if (VT_hientai.NV==0):
		if (VT_hientai.DK == 9):  # Khong con nhiem vu, co lenh lui - tai tram sac
			LuiXe()
		# ===================#
		elif (VT_hientai.DK == 1):  # Khong con nhiem vu, co lenh RE PHAI
			RePhai()
		# ===================#
		elif (VT_hientai.DK == 2):  # Khong con nhiem vu, co lenh RE TRAI
			ReTrai()
		# ===================#
		elif (VT_hientai.DK == 3):  # Khong con nhiem vu, co lenh XOAY NGUOC LAI
			Quay180()
		# ===================#
		elif (VT_hientai.DK == 4):  # DUNG LAI tai VT sac pin
			DungXe()
		elif (VT_hientai.DK == 0):  # Khong con nhiem vu, di tiep
			# print "Test"
			TiepTucDi()
	# ============ XU LY TAI CAC DOAN QUAY/LUI =======================#
	if chaytiep == 1:
		if (ck_turn == 1):
			V = 0
			if (VT_hientai.DK == 2) or (VT_hientai.DK == 3):  # Re Trai
				W = W + 0.002
				if W > 0.5 * Wmax:
					W = 0.5 * Wmax
			if VT_hientai.DK == 1:  # Re phai
				W = W - 0.002
				if W < -0.5 * Wmax:
					W = -0.5 * Wmax
		elif (ck_lui == 1):
			W = 0
			V = V - 0.002
			if V < -Vslow2:
				V = -Vslow2

def KiemTra_MatVachTu():
	global ngoai_vachtu
	global cnt_bamvachtu
	global auto_on
	if ngoai_vachtu == 1:
		cnt_bamvachtu = cnt_bamvachtu+1
		if (cnt_bamvachtu > 1000):  # Sau 20s neu van nam ngoai vach tu thi chuyen sang che do dk bang tay
			auto_on = 0 # chuyen ve bang tay
	else:
		cnt_bamvachtu = 0

def update_status():
	global msg_status
	global auto_on
	global Bumper_Back, Bumper_Front, Front_Obs, Back_Obs
	global ngoai_vachtu
	global Estop
	global TaskList, Sac
	global pose_ht, next_ID
	vatcan = Bumper_Back or Bumper_Front or Front_Obs or Back_Obs
	msg_status = '{0:01d}'.format(int(auto_on))+'{0:01d}'.format(int(vatcan))+'{0:01d}'.format(int(ngoai_vachtu))+'{0:01d}'.format(int(Estop)) 
	#msg_status += '{0:010d}'.format(int(next_ID))
	if len(TaskList)>1:
	#	next_ID = TaskList[1].ID
		msg_status += '{0:010d}'.format(int(next_ID))
	else:
		msg_status += '{0:010d}'.format(pose_ht.position.x)

#================================= MAIN LOOP =============================#
#=========================================================================#
def MAIN_LOOP():
	global dx
	global dr
	global mode
	global dm
	global chaytiep
	global VT_hientai
	global nVT
	global ser
	global lenc
	global renc
	global L
	global R
	global idle
	global TaskList
	global jobFinished
	global jobStart
	global cuaStatus
	global DIR
	global turn
	global CB
	global Laser
	global Laser0
	global vatcan
	global gotTask
	global sound_client
	global VT_hientai
	global lui
	global ngoai_vachtu
	global LastID
	global homeID
	global pub_pose
	global pose_ht
	global started
	global count
	global NV_List
	global passed_rfid
	global V
	global W
	global tranhduong
	global light
	global complete
	global sigma
	global da_layXe
	global layxeID
	global load_xe
	global pub_dir
	global is_mangxe
	global N_ID
	global VT_Sac
	global addr_Front_Obs   # Can truoc
	global addr_Back_Obs	# Can sau
	global addr_Btn_Present	# Co num an chay
	global addr_Estop	# T/h co su co
	global addr_Error	# Co su co
	global addr_run		# Chay va dung (1 - run, 0 - stop)
	global addr_Lift_Up 	# Lenh nang/ha (nang = 1, ha = 0)
	global addr_Lift_Dn
	global addr_load
	global Vfast
	global Vslow1
	global Vslow2
	global dI
	global KP
	global KI
	global dImax
	global Wmax
	global PLC
	global Front_Obs
	global Back_Obs
	global Bumper_Front
	global Bumper_Back
	global left
	global right
	global NV
	global addr_dk_motor
	global ck_connection
	global vibot_state
	global addr_ID
	global addr_Mang_xe
	global addr_NV
	global addr_Hoanthanh_NV
	global addr_DK
	global addr_OLD_ID
	global quaylai_ok
	global ck_quay
	global is_reset
	global addr_PC_treo
	global addr_reset_cam
	global auto_on
	global addr_Bumper_Front
	global addr_Bumper_Back
	global N_ID
	global msg_status
	left = 0
	right = 0
	Tmax = 0
	#============== KHOI TAO CAC THAM SO HE THONG =========================#
	INIT()
	#INIT_PATH_HOC_VIEN()
	INIT_PATH_BENH_VIEN()
	INIT_UDP()

	#============== THAM SO ROS ============================================#
	rospy.init_node("DK_UGV_BASE")
	sound_client = SoundClient()
	pub_lmotor = rospy.Publisher('lwheel', Int64, queue_size=1)
	pub_rmotor = rospy.Publisher('rwheel', Int64, queue_size=1)
	#pub_status = rospy.Publisher('base_moving', Byte, queue_size=1)
	#pub_pose   = rospy.Publisher('/covidbot/observer_path', Pose, queue_size=5)
	#pub_status = rospy.Publisher('robot_status', String, queue_size = 5)
	#pub_light  = rospy.Publisher('/warning_lights', Int8, queue_size=5)
	#pub_dir  = rospy.Publisher('/table_dir', Bool, queue_size=1)
	#rospy.Subscriber('cmd_vel', Twist, twistCallback)
	rospy.Subscriber('vachtu', Float32, getVachTu)
	#rospy.Subscriber('/feedback_cb', cambien, getFeedback)
	#rospy.Subscriber('laser80', Float32, getLaser)
	rospy.Subscriber('ngoai_vachtu', Int16, getNgoaiVachtu)
	#rospy.Subscriber('auto', Int32, autoMode)
	#rospy.Subscriber('manual', Int32, manualMode)
	rospy.Subscriber('rfid_reader', Int64, getRFID)
	#rospy.Subscriber('/covidbot/mode_quayphim', Int32, GetCT)
	rospy.Subscriber('/covidbot/cmd_path', PoseArray, getCmdPath)
	#rospy.Subscriber('battery',BatteryState,getBAT)
	#rospy.Subscriber('/covidbot/lift_up_dn', Int32, getUD)
	rospy.Subscriber('/joy', Joy, MANUAL_JOY)
	rospy.Subscriber('battery',BatteryState,getBAT)

	idle = rospy.Rate(50)

	#============== KET NOI DRIVER DK DONG CO ==============================#
	ser = serial.Serial('/dev/ttyS2',115200)

	# Thiet lap ket noi voi PLC
	PLC = minimalmodbus.Instrument('/dev/ttyS0',5)  # port name, slave address (in decimal)
	PLC.serial.baudrate = 115200         # Baud
	PLC.serial.parity   = minimalmodbus.serial.PARITY_EVEN
	PLC.serial.timeout  = 5          # seconds0
	PLC.mode = minimalmodbus.MODE_RTU   # rtu or ascii mode MODE_ASCII, MODE_RTU
	# Dinh nghia cac dia chi
	MRegBaseAddr = 0x800		#Dia chi co so vung nho
	addr_Front_Obs = MRegBaseAddr + 50      # Can truoc
	addr_Back_Obs = MRegBaseAddr + 51	# Can sau
	addr_Btn_Present = MRegBaseAddr + 52	# Co num an chay
	addr_Estop = MRegBaseAddr + 53		# T/h co su co
	addr_load = MRegBaseAddr + 54
	addr_Error = MRegBaseAddr + 100		# Co su co
	addr_run = MRegBaseAddr + 101		# Chay va dung (1 - run, 0 - stop)
	addr_Lift_Up = MRegBaseAddr + 102	# Lenh nang
	addr_Lift_Dn = MRegBaseAddr + 103	# Lenh ha
	addr_dk_motor = MRegBaseAddr + 63
	addr_reset_cam = MRegBaseAddr + 105
	addr_PC_treo = MRegBaseAddr + 56
	addr_Bumper_Front = MRegBaseAddr + 61
	addr_Bumper_Back = MRegBaseAddr + 62
	

	# Dia chi vung nho de luu trang thai cua robot tren PLC
	addr_ID =  4096 + 408
	addr_Mang_xe = 4096 + 410
	addr_NV      = 4096 + 412
	addr_Hoanthanh_NV = 4096 + 414
	addr_DK = 4096 + 416
	addr_OLD_ID = 4096 + 418

	N = len(TaskList)
	for i in range(0,N):
		print TaskList[i].ID, TaskList[i].DK, TaskList[i].NV
	#========================= TIMER ========================================#
	Timer_rw = QQTimer(0.02,PLC_rw)
	Timer_rw.start()

	Timer_enc = QQTimer(0.03,getEncoder)
	Timer_enc.start()

	Timer_SpeedCmd = QQTimer(0.05,sendSpeedCmd)
	Timer_SpeedCmd.start()
	tmp_chaytiep = 1
	###================== MAIN LOOP =======================================###
	###====================================================================###

    # DOC LAI TRANG THAI CU CUA ROBOT
	reset()
	cnt_ck = 0
	while not rospy.is_shutdown():
		cnt_ck = cnt_ck + 1
		if cnt_ck >50:
			#print "auto_on:", auto_on, "gotTask:", gotTask, "chaytiep:", chaytiep, "ck_lui:", ck_lui
			cnt_ck=0	
		#================ CHE DO DK BANG TAY =================================#
		if auto_on==0:
			chaytiep = 1
			V = dx
			W = dr
		#=================CHE DO TU DONG  ==================================#
		elif auto_on==1:
			KiemTra_MatVachTu()
			if gotTask: # co tastlist de thuc hien
				main_Move()
				main_NV()
				main_DK()
			if (len(TaskList)==0) and (VT_hientai.NV != 4): # xu ly truong hop diem cuoi khong thuc hien NV dung
				chaytiep = 0
				if gotTask:	
					sound_client.playWave("/opt/ros/melodic/share/sound_play/sounds/hoan_thanh_nhiem_vu.wav")
					VT_hientai.ID = 0  # Reset lai Task
					gotTask = 0				
					V = 0
					W = 0
					dI = 0
					NV = 0
		#================ CHE DO TU DONG BAM DUONG KHI GAP VACH TU ==========#
		else:
			if N_ID==1: # tro lai che do bang tay
				auto_on = 0
			else:
				main_Move_BamVach()
		#==================TIEP TUC CHUONG TRINH CHINH ===========================================#
		if chaytiep==0:
			V = V - 0.03
			V = max(0,V)
			W = 0
		right = 63.6619772368*(V + W*L)/R #
		left  = 63.6619772368*(V - W*L)/R #
		#=========== PULISH ENCODER LEN MANG ROS =================================================#
		pub_lmotor.publish(lenc)
		pub_rmotor.publish(renc)
		#pub_pose.publish(pose_ht)
		#================= CAP NHAT TRANG THAI ROBOT ====================#		
		update_status()  # 
		AUTO_SEND_FEEDBACK()
		
		idle.sleep()

#====================================== MAIN PROGRAM ============================#
if __name__ == '__main__':
	try:
		MAIN_LOOP()
	except rospy.ROSInterruptException:
		pass

