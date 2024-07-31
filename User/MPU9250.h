
// 定義MPU9250內部位址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺儀取樣速率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通濾波頻率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺儀自檢及測量範圍，典型值：0x18(不自檢，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速計自檢、測量範圍及高通濾波頻率，典型值：0x01(不自檢，2G，5Hz)

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define	INT_CONFIG		0x37
#define	INT_EN				0x38
#define	PWR_MGMT_1		0x6B	//電源管理，典型值：0x00(正常啟用)
#define	WHO_AM_I		  0x75	//IIC位址寄存器(默認數值0x68，唯讀)


//****************************

#define	GYRO_ADDRESS   0x68	  //陀螺地址
#define MAG_ADDRESS    0x0C   //磁場地址
#define ACCEL_ADDRESS  0x68 
