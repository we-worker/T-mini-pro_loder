# 1 扫描命令 [A5 60]
# 当外部设备向 T-mini Pro 发送扫描命令[A5 60]时，T-mini Pro 会进入扫描模式，并反馈点云
# 数据。其应答报文为：
# 起始标志A5 5A 应答长度05 00 应答模式 00 40 类型码81 应答内容....

# 其中第 6 个字节高 2 为 01，因此应答模式取值为 0x1，为持续应答，忽略应答长度，类
# 型码为 0x81；
# 应答内容为系统扫描的点云数据，其按照以下数据结构，以16进制向串口发送至外部设
# 备。
# PH CT FSA LSA CS S1 S2 
# PH(2B) 数据包头 长度为 2B（2字节），固定为 0x55AA，低位在前，高位在后。
# CT(1B) 扫描频率&包类型,表示当前数据包的类型，CT[bit(0)]=1 表示为一圈数据起始，
# CT[bit(0)]=0 表示为点云数据包，CT[bit(7:1)]为预留位。
# LSN(1B) 采样数量 表示当前数据包中包含的采样点数量；起始数据包中只有 1 个起始点的数据，该值为 1。
# FSA(2B) 起始角 采样数据中第一个采样点对应的角度数据
# LSA(2B) 结束角 采样数据中最后一个采样点对应的角度数据
# CS(2B) 校验码 当前数据包的校验码，采用双字节异或对当前数据包进行校验
# Si(3B) 采样数据 S 节点，用于存放系统测距的采样数据和 Quality 数据
# 起始位解析
# 当检测到 CT[bit(0)]=1 时，表明该包数据为起始数据包，表示一圈数据的起始，该数据
# 包中 LSN = 1，即 Si 的数量为 1； 其距离、角度的具体值解析参见下文；
# 当检测到 CT[bit(0)]=0 时，表明该包数据为点云数据包。
# 注：CT[bit(7:1)]为预留位，未来版本会用作其他用途，因此在解析 CT 过程中，只需要对 bit(0)位做起始帧
# 的判断。

# 光强解算公式：𝐼𝑛𝑡𝑒𝑛𝑠𝑖𝑡𝑦(𝑖) = 𝐼𝑛𝑡𝑒𝑛𝑠𝑖𝑡𝑦[7: 0]
# 光强表示该激光信号的强弱信息，存放在 Si 节点的第 1 个字节中。
# 其中， Si 为采样数据，Si(j)表示 Si 的第 j 个字节。设采样数据为 64 E5 6F，所以本采样点 S(1) = 0x64=100(dec)，所以 Intensity= 100。
# 注：S 节点中， 第一个字节存放 Intensity，其取值范围为 0~255。

# 距离解算公式：Distance 𝑖 = 𝐿𝑠ℎ𝑖𝑓𝑡𝑏𝑖𝑡(𝑆𝑖(3), 6) + 𝑅𝑠ℎ𝑖𝑓𝑡𝑏𝑖𝑡(𝑆𝑖(2), 2)
# 其中， Si 为采样数据，Lshiftbit(data, 1)表示将数据 data 左移一位，Rshiftbit(data, 1)表示将数据 data 右移一位。设采样数据为 64 E5 6F，由于本系统是小端模式，所以本采样
# 点求解得：Distance = dec(𝐿𝑠ℎ𝑖𝑓𝑡𝑏𝑖𝑡(0x6F, 6)) + dec(𝑅𝑠ℎ𝑖𝑓𝑡𝑏𝑖𝑡(0𝑥𝐸5,2)) = 7161𝑚𝑚

# 角度数据保存在 FSA 和 LSA 中，每一个角度数据有如下的数据结构，C 是校验位，其值固定为 1。具体过程如下：
# 起始角解算公式：𝐴𝑛𝑔𝑙𝑒_𝐹𝑆𝐴 = 𝑅𝑠ℎ𝑖𝑓𝑡𝑏𝑖𝑡(𝐹𝑆𝐴,1)/64
# 结束角解算公式：𝐴𝑛𝑔𝑙𝑒_𝐿𝑆𝐴 = 𝑅𝑠ℎ𝑖𝑓𝑡𝑏𝑖𝑡(𝐿𝑆𝐴,1)/64
# 中间角解算公式：𝐴𝑛𝑔𝑙𝑒 𝑖 =𝑑𝑖𝑓𝑓(𝐴𝑛𝑔𝑙𝑒)/(𝐿𝑆𝑁−1)∗ (𝑖 − 1) + 𝐴𝑛𝑔𝑙𝑒_𝐹𝑆𝐴  (𝑖 = 2,3, … , 𝐿𝑆𝑁 − 1)
# 𝑅𝑠ℎ𝑖𝑓𝑡𝑏𝑖𝑡(𝑑𝑎𝑡𝑎, 1)表示将数据 data 右移一位。𝑑𝑖𝑓𝑓(𝐴𝑛𝑔𝑙𝑒)表示起始角到结束角的顺时针角度差，LSN 表示本帧数据包采样数量。


import serial

import matplotlib.pyplot as plt
import numpy as np


class LidarDriver:
	def __init__(self, port, baudrate=230400):
		self.port = port
		self.baudrate = baudrate
		self.ser = None

		self.dist_angles = dict()
		self.tempdistances=list()
		self.tempangles=list()

	def open_port(self):
		self.ser = serial.Serial(self.port, self.baudrate)

	def close_port(self):
		if self.ser:
			self.ser.close()

	def send_scan_command(self):
		scan_command = [0xA5, 0x60]
		self.ser.write(bytearray(scan_command))

	def read_and_parse_data(self):
		while True:
			# 从串口读取报文头部
			# header = self.ser.read(7)

			# # 检查起始标志
			# if header[0:2] != b'\xA5\x5A':
			# 	print("Invalid start flag")
			# 	return

			# # 获取应答长度
			# response_length = int.from_bytes(header[2:4], byteorder='little')

			# # 获取应答模式
			# response_mode = (header[5] & 0xC0) >> 6
			response_mode = 1

			# 如果应答模式为持续应答，读取数据直到遇到新的包头
			if response_mode == 0x1:
				response_content = b''
				while True:
					byte = self.ser.read()
					response_content += byte
					if len(response_content) >= 4 and response_content[-2:] == b'\xAA\x55':
						# 当遇到新的包头时，将已读取的数据传递给parse_response方法进行解析
						self.parse_response(response_content[:-2])
						# 将新的包头作为下一个数据包的头部
						# header = response_content[-2:]
						response_content = b'\xAA\x55'
			else:
				# 从串口读取剩余的报文
				response_content = self.ser.read(response_length)
				# 调用parse_response方法进行解析
				self.parse_response(response_content)


	def parse_response(self, response):
		# 将响应转换为字节
		response_bytes = bytearray(response)

		# 解析数据包头
		PH = response_bytes[0:2]
		print(f"Data Packet Header: {PH}")

		# 解析扫描频率和包类型
		CT = response_bytes[2]
		print(f"Scan Frequency & Packet Type: {CT}")

		# 如果是新的一帧数据，保存上一帧的数据并清空临时列表
		if CT & 0x01:
			self.dist_angles['ranges'] = self.tempdistances
			self.dist_angles['angles'] = self.tempangles
			self.dist_angles['is_new'] = True
			self.tempdistances = []
			self.tempangles = []
			self.plot_scan_data()

		# 解析采样数量
		LSN = response_bytes[3]
		print(f"Sample Quantity: {LSN}")

		# 解析起始角
		FSA = int.from_bytes(response_bytes[4:6], byteorder='little')
		Angle_FSA = (FSA >> 1) / 64.0
		print(f"Start Angle: {Angle_FSA}")

		# 解析结束角
		LSA = int.from_bytes(response_bytes[6:8], byteorder='little')
		Angle_LSA = (LSA >> 1) / 64.0
		print(f"End Angle: {Angle_LSA}")

		# 解析校验码
		CS = response_bytes[8:10]
		print(f"Check Code: {CS}")

		# 解析采样数据
		Si = response_bytes[10:]

		# 根据LSN进行数据读取
		for i in range(LSN):
			# 3个字节为一组数据
			data_index = i * 3

			# 解析光强
			Intensity = Si[data_index]
			print(f"Intensity {i}: {Intensity}")

			# 解析距离
			Distance = (Si[data_index + 2] << 6) + (Si[data_index + 1] >> 2)
			print(f"Distance {i}: {Distance}")
			self.tempdistances.append(Distance)

			# 计算角度
			if(LSN==1):
				Angle_i=Angle_LSA
			else:
				diff_Angle = Angle_LSA - Angle_FSA if Angle_LSA > Angle_FSA else Angle_LSA + 360 - Angle_FSA
				Angle_i = diff_Angle / (LSN - 1) * i + Angle_FSA
			print(f"Angle {i}: {Angle_i}")
			self.tempangles.append(Angle_i)
	def plot_scan_data(self):
		# 将角度和距离分开
		angles=self.dist_angles['angles']
		distances=self.dist_angles['ranges']

		# 将角度从度转换为弧度
		angles = np.deg2rad(angles)
		global ax,lines
		if('line' in locals()):
			line.remove()
		line = ax.scatter(angles, distances, c="pink", s=5)
		ax.set_theta_offset(3.1415926 / 2)
		plt.pause(0.01)


fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection="polar")
ax.set_title('lidar (exit: Key E)',fontsize=18)
ax.set_theta_direction(-1)  # 顺时针
plt.connect('key_press_event', lambda event: exit(1) if event.key == 'e' else None)

lines = list()
ld = LidarDriver('/dev/ttyUSB0')
ld.open_port()
ld.send_scan_command()
ld.read_and_parse_data()
