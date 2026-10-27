#include "lcd_init.h"
#include "stm32f4xx.h"
void delay_ms(uint16_t time);
// SPI句柄
SPI_HandleTypeDef hspi1;
void delay_ms(uint16_t time);

/******************************************************************************
	  函数说明：LCD GPIO 和 硬件SPI 初始化
	  修改说明：PA5, PA7 改为复用功能，PA4, PA6, PA8, PA11 保持推挽输出
******************************************************************************/
void LCD_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	// 1. 开启时钟
	__HAL_RCC_GPIOA_CLK_ENABLE(); // 开启GPIOA时钟
	__HAL_RCC_SPI1_CLK_ENABLE();  // 【新增】开启SPI1时钟

	// 2. 配置SPI引脚 (PA5=SCK, PA7=MOSI) 为复用推挽输出
	GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;			  // 复用推挽
	GPIO_InitStructure.Pull = GPIO_NOPULL;				  // 上拉或浮空均可，通常SPI由主机驱动
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 高速
	GPIO_InitStructure.Alternate = GPIO_AF5_SPI1;		  // 指定复用为SPI1
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 3. 配置控制引脚 (PA4=RES, PA6=DC, PA8=CS, PA11=BLK) 为普通推挽输出
	// 注意：PA6虽然是SPI1_MISO，但LCD不需要读数据，且被用作DC脚，所以配置为普通GPIO
	GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_11;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; // 普通推挽
	GPIO_InitStructure.Pull = GPIO_PULLUP;		   // 上拉
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Alternate = 0; // 不使用复用
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 4. 控制引脚默认电平
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_11, GPIO_PIN_SET);

	// 5. 【新增】SPI1 参数配置
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;			 // 主机模式
	hspi1.Init.Direction = SPI_DIRECTION_2LINES; // 双线全双工 (虽然只用发送，但这是标准配置)
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;	 // 8位数据
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;	 // CPOL=1 (空闲时高电平，ST7789常用)
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;		 // CPHA=2 (第二个边沿采样，模式3)
	// 注意：如果屏幕显示花屏，尝试改成 POLARITY_LOW 和 PHASE_1EDGE (模式0)

	hspi1.Init.NSS = SPI_NSS_SOFT; // 软件控制CS (我们自己操作PA8)

	// SPI速度设置：
	// F407 APB2总线频率84MHz。
	// 分频系数2 => 42MHz (屏幕可能受不了，线长容易出错)
	// 分频系数4 => 21MHz (推荐，稳定)
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;

	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB; // 高位先出
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;

	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		// 初始化错误处理
		while (1)
			;
	}
}

// /******************************************************************************
//       函数说明：LCD串行数据写入函数
//       入口数据：dat  要写入的串行数据
//       返回值：  无
// ******************************************************************************/
// void LCD_Writ_Bus(uint8_t dat) 
// {	
// 	uint8_t i;
// 	LCD_CS_Clr();
// 	for(i=0;i<8;i++)
// 	{			  
// 		LCD_SCLK_Clr();
// 		if(dat&0x80)
// 		{
// 		   LCD_MOSI_Set();
// 		}
// 		else
// 		{
// 		   LCD_MOSI_Clr();
// 		}
// 		LCD_SCLK_Set();
// 		dat<<=1;
// 	}	
//   LCD_CS_Set();	
// }
/******************************************************************************
// 	  函数说明：LCD写入数据
// 	  入口数据：dat 写入的数据
// 	  返回值：  无
// ******************************************************************************/
// void LCD_WR_DATA8(uint8_t dat)
// {
// 	LCD_Writ_Bus(dat);
// }

// /******************************************************************************
// 	  函数说明：LCD写入数据
// 	  入口数据：dat 写入的数据
// 	  返回值：  无
// ******************************************************************************/
// void LCD_WR_DATA(uint16_t dat)
// {
// 	LCD_Writ_Bus(dat >> 8);
// 	LCD_Writ_Bus(dat);
// }

/******************************************************************************
	  函数说明：LCD串行数据写入函数 (硬件SPI版本)
	  入口数据：dat  要写入的串行数据
	  返回值：  无
******************************************************************************/
void LCD_Writ_Bus(uint8_t dat)
{
	// 使用HAL库发送一个字节
	LCD_CS_Clr(); // 拉低片选选中LCD

	// 发送数据：&hspi1是句柄，&dat是数据地址，1是长度，100是超时时间(ms)
	HAL_SPI_Transmit(&hspi1, &dat, 1, 100);

	LCD_CS_Set(); // 拉高片选结束
}
/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}

/******************************************************************************
	  函数说明：LCD写入数据
	  入口数据：dat 写入的数据
	  返回值：  无
******************************************************************************/
void LCD_WR_DATA(uint16_t dat)
{
	uint8_t data[2];
	data[0] = dat >> 8;
	data[1] = dat;

	// 连续发送两个字节，减少 CS 翻转
	LCD_CS_Clr();
	HAL_SPI_Transmit(&hspi1, data, 2, 100);
	LCD_CS_Set();
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+20);
		LCD_WR_DATA(y2+20);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+20);
		LCD_WR_DATA(y2+20);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+20);
		LCD_WR_DATA(x2+20);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+20);
		LCD_WR_DATA(x2+20);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
}

void LCD_Init(void)
{
	LCD_GPIO_Init();//初始化GPIO
	
	LCD_RES_Clr();//复位
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);
	
	LCD_BLK_Set();//打开背光
    delay_ms(100);
	
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	delay_ms(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);			
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);			
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C); 
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x33); 
	LCD_WR_DATA8(0x33); 			

	LCD_WR_REG(0xB7);			
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xBB);			
	LCD_WR_DATA8(0x32); //Vcom=1.35V
					
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);			
	LCD_WR_DATA8(0x15); //GVDD=4.8V  颜色深度
				
	LCD_WR_REG(0xC4);			
	LCD_WR_DATA8(0x20); //VDV, 0x20:0v

	LCD_WR_REG(0xC6);			
	LCD_WR_DATA8(0x0F); //0x0F:60Hz        	

	LCD_WR_REG(0xD0);			
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1); 

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x05);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);   

	LCD_WR_REG(0xE1);     
	LCD_WR_DATA8(0xD0);   
	LCD_WR_DATA8(0x08);   
	LCD_WR_DATA8(0x0E);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x09);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x48);   
	LCD_WR_DATA8(0x17);   
	LCD_WR_DATA8(0x14);   
	LCD_WR_DATA8(0x15);   
	LCD_WR_DATA8(0x31);   
	LCD_WR_DATA8(0x34);
	LCD_WR_REG(0x21); 

	LCD_WR_REG(0x29);
} 



void delay_ms(uint16_t time){
	HAL_Delay(time);
}




