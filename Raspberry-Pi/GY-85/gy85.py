from wiringpi import wiringPiSetup, wiringPiI2CSetup, wiringPiI2CWriteReg8, wiringPiI2CReadReg8, delay
from math import sqrt, atan2, pi

#wiringPiSetup()

ADXL345=0x53
DATAX0 =0x32
HMC5883=0x1E
ITG3200=0x68

class gy85():

	def __init__(self):
		self.setupAcc()
		self.setupCompass()
		self.setupGyro()
		self.alpha=0.5
		self.Acc={'Roll':0.0, 'Pitch':0.0}
		self.Fg=[0.0,0.0,0.0]
		self.Gy={'Roll':0.0, 'Pitch':0.0, 'Yaw':0.0}
		self.Ma={'Yaw':0.0}

	def setupAcc(self):
		self.acc=wiringPiI2CSetup(ADXL345)
		if(self.acc == -1):
			print "Error al conectar con ADXL345"
			return -1  
		#Turning on the ADXL345
		wiringPiI2CWriteReg8(self.acc, 0x31, 0x01)      
		wiringPiI2CWriteReg8(self.acc, 0x2d, 0x08)
		return 0
	
	def readAcc(self):
		acc=self.acc
		buff=[0,0,0,0,0,0]
		axis=[0,0,0]
		for i in range(6):
			buff[i]=wiringPiI2CReadReg8(acc, DATAX0+i)
		
		axis[0] = ((buff[1]) << 8) | buff[0]
		axis[1] = ((buff[3]) << 8) | buff[2]
		axis[2] = ((buff[5]) << 8) | buff[4]
		
		return map(lambda x: x * 0.0039, axis)

	def setupCompass(self):
		self.comp=wiringPiI2CSetup(HMC5883)
		if(self.comp == -1):
			print "Error al conectar con HMC5883"
			return -1  
		#Turning on the ADXL345
		wiringPiI2CWriteReg8(self.comp, 0x02, 0x00)      
		return 0

	def readCompass(self):
		comp=self.comp
		buff=[0,0,0,0,0,0]
		axis=[0,0,0]
		for i in range(6):
			buff[i]=wiringPiI2CReadReg8(comp, 0x03+i)
		
		axis[0] = buff[0]<<8
		axis[0] |= buff[1]
		axis[2] = buff[2]<<8
		axis[2] |= buff[3]
		axis[1] = buff[4]<<8
		axis[1] |= buff[5]

		return map(lambda x: x * 0.92, axis)

	def setupGyro(self):
		self.gy=wiringPiI2CSetup(ITG3200)
		if(self.gy == -1):
			print "Error al conectar con ITG3200"
			return -1  
		#Turning on the ITG3200
		wiringPiI2CWriteReg8(self.gy, 0x3e, 0x00)      
		wiringPiI2CWriteReg8(self.gy, 0x15, 0x07)
		wiringPiI2CWriteReg8(self.gy, 0x16, 0x1e)
		wiringPiI2CWriteReg8(self.gy, 0x17, 0x00)

		delay(10)

		self.gyroCalibrate()

	def gyroCalibrate(self):
		tmpx=0
		tmpy=0
		tmpz=0
		self.g_offx=0
		self.g_offy=0
		self.g_offz=0

		for i in range(10):
			delay(10)
			gp=self.readGyro()
			tmpx=tmpx+gp[0]
			tmpy=tmpy+gp[1]
			tmpz=tmpz+gp[2]
		
		self.g_offx=tmpx/10
		self.g_offy=tmpy/10
		self.g_offz=tmpz/10

	def readGyro(self):
		buff=[0,0,0,0,0,0,0,0]
		axis=[0,0,0,0]
		for i in range(8):
			buff[i]=wiringPiI2CReadReg8(self.gy, 0x1b+i)

		axis[0] = ((buff[2] << 8) | buff[3]) - self.g_offx
		axis[1] = ((buff[4] << 8) | buff[5]) - self.g_offy
		axis[2] = ((buff[6] << 8) | buff[7]) - self.g_offz
		axis[3] = ((buff[0] << 8) | buff[1])

		return axis

	def readAngles(self):
		G=self.getGyAngles()
		A=self.getAccAngles()
		M=self.getMagAngles()

		return A,G,M
	

	def getGyAngles(self):
		dtime=20
		V= self.readGyro()
		for i in range(3):
			V[i]=float(V[i])/14.375
		V[3]=35.0+(float(V[3])+13200)/280.0
		
		Roll, Pitch, Yaw= self.Gy['Roll'], self.Gy['Pitch'], self.Gy['Yaw']

		Roll= Roll - V[0] * dtime * 0.001
		Pitch= Pitch + V[1] * dtime* 0.001
		Yaw= Yaw - V[2] * dtime * 0.001

		self.Gy['Roll'], self.Gy['Pitch'], self.Gy['Yaw'] = Roll, Pitch, Yaw

		return self.Gy

	def getAccAngles(self):
		G= self.readAcc()
		alpha=self.alpha
		Roll, Pitch, Fg=self.Acc['Roll'], self.Acc['Pitch'], self.Fg

		#Calculos a partir del Acelerometro
		Fg=map(lambda x,y: x * alpha + y * (1.0 - alpha), G, Fg)
		Roll=atan2(Fg[1],Fg[2]) * 180/pi
		Pitch=atan2(-1*Fg[0],sqrt(Fg[1]*Fg[1]+Fg[2]*Fg[2])) * 180/pi
		
		self.Acc['Roll'], self.Acc['Pitch'], self.Fg = Roll, Pitch, Fg
		
		return self.Acc

	def getMagAngles(self):
		M= self.readCompass()
		Yaw= atan2(M[1],M[0])
		if Yaw < 0:
			Yaw = Yaw + 2*pi
		if Yaw > 2*pi:
			Yaw = Yaw - 2*pi 
		Yaw = Yaw * 180/pi 
		self.Ma['Yaw'] = Yaw

		return self.Ma