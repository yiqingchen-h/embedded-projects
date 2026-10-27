  #include "IIC.h"
// //打开SDA引脚（输出）
// void SDA_OUT(void)   
// {
//     DL_GPIO_initDigitalOutput(GPIO_sda_PIN_0_IOMUX);     
// 	DL_GPIO_setPins(GPIO_sda_PORT, GPIO_sda_PIN_0_PIN);	   
//     DL_GPIO_enableOutput(GPIO_sda_PORT, GPIO_sda_PIN_0_PIN); 
// }
// //关闭SDA引脚（输入）
// void SDA_IN(void)
// {
 
//     DL_GPIO_initDigitalInputFeatures(GPIO_sda_PIN_0_IOMUX,
// 		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
// 		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
 
 
// }
 
// void Delay_us(uint16_t us)
// {
//     while(us--)
//     delay_cycles(CPUCLK_FREQ/1000000);
// }//CPUCLK_FREQ为时钟频率，可以根据配置的改变而改变
// /*引脚配置层*/
 
// /**
//   * 函    数：I2C写SCL引脚电平
//   * 参    数：BitValue 协议层传入的当前需要写入SCL的电平，范围0~1
//   * 返 回 值：无
//   * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SCL为低电平，当BitValue为1时，需要置SCL为高电平
//   */
// void MyI2C_W_SCL(uint8_t BitValue)
// {
//     if(BitValue)
//         DL_GPIO_setPins(GPIO_scl_PORT, GPIO_scl_PIN_1_PIN);
//     else
//         DL_GPIO_clearPins(GPIO_scl_PORT, GPIO_scl_PIN_1_PIN);
// 	Delay_us(8);	//延时8us，防止时序频率超过要求
// }
 
// /**
//   * 函    数：I2C写SDA引脚电平
//   * 参    数：BitValue 协议层传入的当前需要写入SDA的电平，范围0~0xFF
//   * 返 回 值：无
//   * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SDA为低电平，当BitValue非0时，需要置SDA为高电平
//   */
// void MyI2C_W_SDA(uint8_t BitValue)
// {
//     SDA_OUT();
//     if(BitValue)
//         DL_GPIO_setPins(GPIO_sda_PORT, GPIO_sda_PIN_0_PIN);
//     else
//         DL_GPIO_clearPins(GPIO_sda_PORT, GPIO_sda_PIN_0_PIN);
// 	Delay_us(8);					//延时8us，防止时序频率超过要求
// }
 
// /**
//   * 函    数：I2C读SDA引脚电平
//   * 参    数：无
//   * 返 回 值：协议层需要得到的当前SDA的电平，范围0~1
//   * 注意事项：此函数需要用户实现内容，当前SDA为低电平时，返回0，当前SDA为高电平时，返回1
//   */
// uint8_t MyI2C_R_SDA(void)
// {
// 	uint8_t b;
//     uint32_t BitValue;
//     SDA_IN();
// 	BitValue = DL_GPIO_readPins(GPIO_sda_PORT, GPIO_sda_PIN_0_PIN);		//读取SDA电平
//     {
//         if(BitValue)   b=1;
//         else           b=0;
//     }
// 	Delay_us(8);		//延时8us，防止时序频率超过要求
// 	return b;	        //返回SDA电平
// }
 
// /**
//   * 函    数：I2C初始化
//   * 参    数：无
//   * 返 回 值：无
//   * 注意事项：此函数需要用户实现内容，实现SCL和SDA引脚的初始化
//   */
// void MyI2C_Init(void)
// {
//     SYSCFG_DL_GPIO_init();
// 	/*设置默认电平*/
// 	DL_GPIO_setPins(GPIOA, GPIO_sda_PIN_0_PIN |GPIO_scl_PIN_1_PIN);//设置PA8和PA9引脚初始化后默认为高电平（释放总线状态）
// }
 
// /*协议层*/
 
// /**
//   * 函    数：I2C起始
//   * 参    数：无
//   * 返 回 值：无
//   */
// void MyI2C_Start(void)
// {
//     SDA_OUT();
// 	MyI2C_W_SDA(1);				//释放SDA，确保SDA为高电平
// 	MyI2C_W_SCL(1);				//释放SCL，确保SCL为高电平
// 	MyI2C_W_SDA(0);				//在SCL高电平期间，拉低SDA，产生起始信号
// 	MyI2C_W_SCL(0);				//起始后拉低SCL，为了占用总线，方便总线时序的拼接
// }
 
// /**
//   * 函    数：I2C终止
//   * 参    数：无
//   * 返 回 值：无
//   */
// void MyI2C_Stop(void)
// {
//     SDA_OUT();
// 	MyI2C_W_SDA(0);							//拉低SDA，确保SDA为低电平
// 	MyI2C_W_SCL(1);							//释放SCL，使SCL呈现高电平
// 	MyI2C_W_SDA(1);							//在SCL高电平期间，释放SDA，产生终止信号
// }
 
// /**
//   * 函    数：I2C发送一个字节
//   * 参    数：Byte 要发送的一个字节数据，范围：0x00~0xFF
//   * 返 回 值：无
//   */
// void MyI2C_SendByte(uint8_t Byte)
// {
//     SDA_OUT();
// 	uint8_t i;
// 	for (i = 0; i < 8; i ++)				//循环8次，主机依次发送数据的每一位
// 	{
// 		MyI2C_W_SDA(Byte & (0x80 >> i));	//使用掩码的方式取出Byte的指定一位数据并写入到SDA线
// 		MyI2C_W_SCL(1);						//释放SCL，从机在SCL高电平期间读取SDA
// 		MyI2C_W_SCL(0);						//拉低SCL，主机开始发送下一位数据
// 	}
// }
 
// /**
//   * 函    数：I2C接收一个字节
//   * 参    数：无
//   * 返 回 值：接收到的一个字节数据，范围：0x00~0xFF
//   */
// uint8_t MyI2C_ReceiveByte(void)
// {
//     SDA_OUT();
// 	uint8_t i, Byte = 0x00;	//定义接收的数据，并赋初值0x00
// 	MyI2C_W_SDA(1);			//接收前，主机先确保释放SDA，避免干扰从机的数据发送
// 	for (i = 0; i < 8; i ++)	//循环8次，主机依次接收数据的每一位
// 	{
//         SDA_IN();
// 		MyI2C_W_SCL(1);						//释放SCL，主机机在SCL高电平期间读取SDA
// 		if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}	//读取SDA数据，并存储到Byte变量
// 	//当SDA为1时，置变量指定位为1，当SDA为0时，不做处理，指定位为默认的初值0
// 		MyI2C_W_SCL(0);						//拉低SCL，从机在SCL低电平期间写入SDA
// 	}
// 	return Byte;							//返回接收到的一个字节数据
// }
 
// /**
//   * 函    数：I2C发送应答位
//   * 参    数：Byte 要发送的应答位，范围：0~1，0表示应答，1表示非应答
//   * 返 回 值：无
//   */
// void MyI2C_SendAck(uint8_t AckBit)
// {
//     SDA_OUT();
// 	MyI2C_W_SDA(AckBit);					//主机把应答位数据放到SDA线
// 	MyI2C_W_SCL(1);							//释放SCL，从机在SCL高电平期间，读取应答位
// 	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序模块
// }
 
// /**
//   * 函    数：I2C接收应答位
//   * 参    数：无
//   * 返 回 值：接收到的应答位，范围：0~1，0表示应答，1表示非应答
//   */
// uint8_t MyI2C_ReceiveAck(void)
// {
//     SDA_OUT();
// 	uint8_t AckBit;							//定义应答位变量
// 	MyI2C_W_SDA(1);							//接收前，主机先确保释放SDA，避免干扰从机的数据发送
// 	MyI2C_W_SCL(1);							//释放SCL，主机机在SCL高电平期间读取SDA
//     SDA_IN();
// 	AckBit = MyI2C_R_SDA();					//将应答位存储到变量里
// 	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序模块
// 	return AckBit;							//返回定义应答位变量
// }

unsigned char old_data_flag = 0;

// 发送一个字节
void Tx_data(unsigned char Id , unsigned char* data)
{
	DL_I2C_fillControllerTXFIFO(I2C_0_INST,data,1);   //硬件IIC写入发送缓冲区
	while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  //等待IIC空闲
	DL_I2C_startControllerTransfer(I2C_0_INST,Id,DL_I2C_CONTROLLER_DIRECTION_TX,1);   //开启发送
	while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));   //等待总线不繁忙
	while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  //等待IIC空闲
	DL_I2C_flushControllerTXFIFO(I2C_0_INST);   //清空发送缓冲区


}

// 发送多个字节
void Tx_datas(unsigned char Id , unsigned char* data,unsigned char szie)
{
	DL_I2C_fillControllerTXFIFO(I2C_0_INST,data,szie);   //硬件IIC写入发送缓冲区
	while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  //等待IIC空闲
	DL_I2C_startControllerTransfer(I2C_0_INST,Id,DL_I2C_CONTROLLER_DIRECTION_TX,szie);   //开启发送
	while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));   //等待总线不繁忙
	while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  //等待IIC空闲
	DL_I2C_flushControllerTXFIFO(I2C_0_INST);   //清空发送缓冲区


}
// 接收一个字节
unsigned char Rx_data(unsigned char Id)
{
	unsigned char bit ;
	DL_I2C_startControllerTransfer(I2C_0_INST,Id,DL_I2C_CONTROLLER_DIRECTION_RX,1);   //开启接收
	while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));   //等待总线不繁忙
	while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  //等待IIC空闲
	bit =  DL_I2C_receiveControllerData(I2C_0_INST);
	return bit;
}

// 接收多个字节
void Rx_datas(unsigned char Id ,unsigned char data[] ,unsigned char size)
{
	DL_I2C_startControllerTransfer(I2C_0_INST,Id,DL_I2C_CONTROLLER_DIRECTION_RX,size);   //开启接收
	while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));   //等待总线不繁忙
	while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  //等待IIC空闲
	for(unsigned char i = 0; i < size ; i++)
	{
		data[i] = DL_I2C_receiveControllerData(I2C_0_INST);
	}
}

 #include "Uart.h"
 // 读取灰度是否正常通讯
unsigned char Ping(void)
{
 	unsigned char dat;
	unsigned char tx[1]= {0xAA};
	Tx_data(Grayscale_ID,tx);
	dat = Rx_data(Grayscale_ID);
	printf("dat:0x%x-PingError\t\n",dat);
	old_data_flag = get_Ping;
	if(dat==0x66){
		return 0;
	}
	return 1;
}

// 读取数字量灰度
unsigned char get_Digital_Output_Grayscale(void)
{
	unsigned char dat;
	//if (old_data_flag != get_Digital_Output)
	{
		unsigned char tx[1]= {0xDD};
		Tx_data(Grayscale_ID,tx);
	}
	dat = Rx_data(Grayscale_ID);

	old_data_flag = get_Digital_Output;	//置标志位 -- 当上一次发送命令为读取数字量时连续读取时可不在发送命令
	return dat;
}
// 读取单通道模拟量灰度
unsigned char get_One_Channel_Analog_Output_Grayscale(unsigned char Channel)
{
	unsigned char dat;
	//if (old_data_flag != One_Channel_Analog)
	{
		unsigned char tx[1]= {0xB0};
		tx[0] |= Channel;
		Tx_data(Grayscale_ID,tx);
	}
	dat = Rx_data(Grayscale_ID);

	old_data_flag = One_Channel_Analog;	//置标志位 -- 当上一次发送命令为读取数字量时连续读取时可不在发送命令
	return dat;
}

// 读取多通道模拟量灰度
unsigned char Channel_Enable = 0;
void get_More_Channel_Analog_Output_Grayscale(unsigned char Channel , unsigned char data[] , unsigned char size)
{
	unsigned char dat;
	//if (old_data_flag != One_Channel_Analog)
	{
		if(Channel_Enable != Channel)
		{
			unsigned char Channel_tx[2]= {0xCE,0xF0};
			Channel_tx[1] = Channel;
			Tx_datas(Grayscale_ID,Channel_tx,2);
			Channel_Enable = Channel;
		}
		unsigned char tx[1]= {0xB0};
		Tx_data(Grayscale_ID,tx);
	}
	Rx_datas(Grayscale_ID,data,size);

	old_data_flag = One_Channel_Analog;	//置标志位 -- 当上一次发送命令为读取数字量时连续读取时可不在发送命令
}



unsigned char AHT10_tx[3] = {0xAC,0x33,0x00};
unsigned char AHT10_rx[6];

void AHT10_Rx(void)
{
	DL_I2C_fillControllerTXFIFO(I2C_0_INST,AHT10_tx,3);   //硬件IIC写入发送缓冲区
	while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  //等待IIC空闲
	DL_I2C_startControllerTransfer(I2C_0_INST,AHT10_ID,DL_I2C_CONTROLLER_DIRECTION_TX,3);   //开启发送
	while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));   //等待总线不繁忙
	while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  //等待IIC空闲
	DL_I2C_flushControllerTXFIFO(I2C_0_INST);   //清空发送缓冲区

	DL_I2C_startControllerTransfer(I2C_0_INST,AHT10_ID,DL_I2C_CONTROLLER_DIRECTION_RX,6);   //开启接收
	while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));   //等待总线不繁忙
	while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  //等待IIC空闲
	for(int i = 0; i < 6 ; i++)
	{
		AHT10_rx[i] = DL_I2C_receiveControllerData(I2C_0_INST);         //读取接收缓冲区
	}
}

#include "math.h"
//AHT10 - 读取温度数据
float AHT10_Temperature(void)
{
	unsigned int Temperature = 0;
	float F_Temperature = 0;
	AHT10_Rx();
	Temperature = (((AHT10_rx[3] & 0x0F) << 16) + ((AHT10_rx[4]) << 8) + AHT10_rx[5]);
	F_Temperature = (Temperature / pow(2,20)) * 200.0 - 50;
	return F_Temperature;
}
//AHT10 - 读取湿度数据
float AHT10_Humidity(void)
{
	unsigned int Humidity = 0;
	float F_Humidity = 0;
	AHT10_Rx();
	Humidity = ((AHT10_rx[1] << 12) + ((AHT10_rx[2]) << 4) + ((AHT10_rx[5] & 0xF0)>>4));
	F_Humidity = (Humidity / pow(2,20)) * 100.0;
	return F_Humidity;
}


