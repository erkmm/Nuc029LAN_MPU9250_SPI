
// �w�qMPU9250������}
//****************************************
#define	SMPLRT_DIV		0x19	//���������˳t�v�A�嫬�ȡG0x07(125Hz)
#define	CONFIG			0x1A	//�C�q�o�i�W�v�A�嫬�ȡG0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//���������ˤδ��q�d��A�嫬�ȡG0x18(�����ˡA2000deg/s)
#define	ACCEL_CONFIG	0x1C	//�[�t�p���ˡB���q�d��ΰ��q�o�i�W�v�A�嫬�ȡG0x01(�����ˡA2G�A5Hz)

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
#define	PWR_MGMT_1		0x6B	//�q���޲z�A�嫬�ȡG0x00(���`�ҥ�)
#define	WHO_AM_I		  0x75	//IIC��}�H�s��(�q�{�ƭ�0x68�A��Ū)


//****************************

#define	GYRO_ADDRESS   0x68	  //�����a�}
#define MAG_ADDRESS    0x0C   //�ϳ��a�}
#define ACCEL_ADDRESS  0x68 
