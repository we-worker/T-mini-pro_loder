

class Lidar_Datas:
	def __init__(self,ser_,dist_angles_):
		self.ser = ser_  # 串口对象，用于读取雷达数据
		self.dist_angles = dist_angles_  # 存储距离和角度数据的字典

		self.tempdistances=list()  # 临时存储距离数据的列表
		self.tempangles=list()  # 临时存储角度数据的列表
		self.packdistances=list()  # 存储一个数据包中的距离数据的列表
		self.packangles=list()  # 存储一个数据包中的角度数据的列表

		self.step = 0  # 当前解析步骤
		self.point_index=0  # 当前解析的点的索引
		self.temp_byte = b''  # 临时存储字节数据

		self.checksum = 0x55AA  # 初始化校验码
		self.CS=0  # 存储读取到的校验码
		self.freq=0 #扫描频率

	def send_scan_command(self):
		#设置为12hz
		scan_command = [0xA5, 0x0B]# 帧数+1
		self.ser.write(bytearray(scan_command))
		scan_command = [0xA5, 0x0B]# 帧数+1
		self.ser.write(bytearray(scan_command))
		scan_command = [0xA5, 0x0B]# 帧数+1
		self.ser.write(bytearray(scan_command))
		scan_command = [0xA5, 0x0B]# 帧数+1
		self.ser.write(bytearray(scan_command))
		scan_command = [0xA5, 0x0B]# 帧数+1
		self.ser.write(bytearray(scan_command))
		scan_command = [0xA5, 0x0B]# 帧数+1
		self.ser.write(bytearray(scan_command))

		#开始扫描
		scan_command = [0xA5, 0x60]
		self.ser.write(bytearray(scan_command))

	def read_and_parse_data(self):
		while True:
			# 从串口读取报文头部
			header = self.ser.read(7)

			# 检查起始标志
			if header[0:2] != b'\x5A\xA5':
				print("初始化失败，可能已经在持续扫描中")
			# 获取应答长度
			response_length = int.from_bytes(header[2:4], byteorder='little')
			# 获取应答模式
			response_mode = (header[5] & 0xC0) >> 6

			# 如果应答模式为持续应答，读取数据直到遇到新的包头
			if response_mode != 0x1:
				# 从串口读取剩余的报文
				for _ in range(response_length):
					byte = self.ser.read(1)
					# 对每个读取的字节进行解析
					self.parse_byte(byte)
			else:
				while True:
					byte = self.ser.read()
					# 对每个读取的字节进行解析
					self.parse_byte(byte)


	def parse_byte(self, byte):
		if self.step == 0:  # 解析数据包头
			if self.temp_byte + byte == b'\xAA\x55':
				self.step += 1
				self.temp_byte=b''
				self.checksum = 0x55AA  # 初始化校验码
			else:
				self.temp_byte = byte
		elif self.step == 1:  # 解析扫描频率和包类型
			CT = int.from_bytes(byte, byteorder='little')
			self.freq=CT
			self.temp_byte+=byte
			if CT & 0x01:
				self.dist_angles['ranges'] = self.tempdistances
				self.dist_angles['angles'] = self.tempangles
				self.dist_angles['is_new'] = True
				self.tempdistances = []
				self.tempangles = []
			self.step += 1
		elif self.step == 2:  # 解析采样数量
			self.LSN =int.from_bytes(byte, byteorder='little')
			self.temp_byte+=byte
			print(f"采样点数: {self.LSN}")
			if self.LSN==1:
				self.freq=(self.freq>>1)/10
				print(f"扫描频率: {self.freq}")
			self.step += 1
			self.checksum ^=int.from_bytes(self.temp_byte, byteorder='little')
			self.temp_byte=b''
		elif self.step in [3, 4]:  # 解析起始角
			self.temp_byte += byte
			if self.step == 4:
				FSA = int.from_bytes(self.temp_byte, byteorder='little')
				self.Angle_FSA = (FSA >> 1) / 64.0
				# print(f"开始角度: {self.Angle_FSA}")
				self.checksum ^=FSA
				self.temp_byte = b''
			self.step += 1
		elif self.step in [5, 6]:  # 解析结束角
			self.temp_byte += byte
			if self.step == 6:
				LSA = int.from_bytes(self.temp_byte, byteorder='little')
				self.Angle_LSA = (LSA >> 1) / 64.0
				# print(f"结束角度: {self.Angle_LSA}")
				self.checksum ^=LSA
				self.temp_byte = b''
			self.step += 1
		elif self.step in [7, 8]:  # 解析校验码
			self.temp_byte += byte
			if self.step == 8:
				self.CS = int.from_bytes(self.temp_byte, byteorder='little')
				# print(f"校验码: {self.CS}")
				self.temp_byte = b''
			self.step += 1
		else:  # 解析采样数据
			self.temp_byte += byte
			if len(self.temp_byte)  == 3:  # 3个字节为一组数据
				self.point_index += 1
				# 更新校验码
				self.checksum ^= self.temp_byte[0]
				self.checksum ^=int.from_bytes(self.temp_byte[1:], byteorder='little')

				Intensity = self.temp_byte[0]
				# print(f"Intensity {self.point_index}: {Intensity}")
				Intensity_Flag=self.temp_byte[1] & 0x03
				# if Intensity_Flag==2 or Intensity_Flag==3:
				if Intensity==0:
					# print("光线强度==0")
					pass
				else:
					Distance = (self.temp_byte[2] << 6) + (self.temp_byte[1] >> 2)
					# print(f"Distance {self.point_index}: {Distance}")
					self.packdistances.append(Distance)

					if(self.LSN==1):
						self.packangles.append(self.Angle_FSA)
					else:
						diff_Angle = self.Angle_LSA - self.Angle_FSA if self.Angle_LSA > self.Angle_FSA else self.Angle_LSA + 360 - self.Angle_FSA
						Angle_i = diff_Angle / (self.LSN - 1) * self.point_index + self.Angle_FSA
						self.packangles.append(Angle_i)
					# print(f"Angle {self.point_index}: {self.packangles[-1]}")
				self.temp_byte = b''

			if self.point_index >= self.LSN:
				self.point_index = 0
				self.step = 0
				if self.checksum==self.CS:
					self.tempdistances.extend(self.packdistances)
					self.tempangles.extend(self.packangles)
					self.packdistances = []
					self.packangles = []
				else:
					self.packdistances = []
					self.packangles = []
					print("!!!!!校验 Error!!!")
					pass




# ===============================================================================
# ====================下面为使用案例===============================================
# ===============================================================================

		 
import serial

import matplotlib.pyplot as plt
import numpy as np
 
from multiprocessing import Process ,Manager

#处理激光雷达数据的进程
def lidar_pross(dist_angles):
	ser = serial.Serial(port='/dev/ttyUSB0',
						baudrate=230400,
						timeout=5.0,
						bytesize=8,
						parity='N',
						stopbits=1)
	lidar=Lidar_Datas(ser, dist_angles)
	lidar.send_scan_command()
	lidar.read_and_parse_data()


line =None
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection="polar")
ax.set_title('lidar (exit: Key E)',fontsize=18)
ax.set_theta_direction(1)  # 逆时针
plt.connect('key_press_event', lambda event: exit(1) if event.key == 'e' else None)

def plot_scan_data(dist_angles):
	# 将角度和距离分开
	angles = dist_angles['angles']
	distances = dist_angles['ranges']
	# 对于范围大于10000的点，略过对应的10范围内的所有点
	# for i in range(len(distances)):
	# 	if distances[i] > 10000:
	# 		for j in range(-10, 10):
	# 			if 0 <= i + j < len(distances):
	# 				distances[i + j] = 0

	# 将角度从度转换为弧度
	angles = np.deg2rad(angles)
	global ax,line
	# 删除上一次的散点图
	if line is not None:
		line.remove()
	line = ax.scatter(angles, distances, c="pink", s=5)
	ax.set_theta_offset(3.1415926 / 2)
	plt.pause(0.01)

# 获取机器人的传感器数据
with Manager() as manager:

	dist_angles = manager.dict()
	dist_angles['ranges'] = list()
	dist_angles['angles'] = list()
	dist_angles['is_new']=False

	lidar = Process(target=lidar_pross, args=(dist_angles,))
	lidar.start()
	while True:
		if dist_angles['is_new']:
			plot_scan_data(dist_angles)
			dist_angles['is_new'] = False