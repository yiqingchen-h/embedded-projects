#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"
#include "hal_data.h"

/******************************************************************************
      函数说明：在指定区域填充颜色
******************************************************************************/
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{          
    uint16_t width = (uint16_t)(xend - xsta);
    uint16_t height = (uint16_t)(yend - ysta);
    
    // 创建一行数据的缓冲区 (RA6M5 RAM 充足，可以直接开辟)
    // 如果宽度很大，可以定义为全局静态变量以节省栈空间
    static uint8_t line_buf[280 * 2]; 
    
    uint8_t c_high = (uint8_t)(color >> 8);
    uint8_t c_low  = (uint8_t)color;

    // 预填充一行缓冲区
    for(int i = 0; i < width; i++)
    {
        line_buf[i*2] = c_high;
        line_buf[i*2+1] = c_low;
    }

    LCD_Address_Set(xsta, ysta, xend-1, yend-1);
    
    LCD_CS_Clr();
    LCD_DC_Set(); // 确保是数据模式
    
    // 逐行发送，每行只触发一次 DMA
    for(int j = 0; j < height; j++)
    {
        LCD_Write_Bus_Large(line_buf, (uint32_t)width * 2);
    }
    
    LCD_CS_Set();
}

/******************************************************************************
      函数说明：在指定位置画点
******************************************************************************/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置 
	LCD_WR_DATA(color);
} 

/******************************************************************************
      函数说明：画线
******************************************************************************/
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
        // 修复警告：强制转换为 uint16_t
		LCD_DrawPoint((uint16_t)uRow, (uint16_t)uCol, color);
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

/******************************************************************************
      函数说明：画矩形
******************************************************************************/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}

/******************************************************************************
      函数说明：画圆
******************************************************************************/
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
        // 修复警告：强制转换为 uint16_t
		LCD_DrawPoint((uint16_t)(x0-b), (uint16_t)(y0-a), color);             //3           
		LCD_DrawPoint((uint16_t)(x0+b), (uint16_t)(y0-a), color);             //0           
		LCD_DrawPoint((uint16_t)(x0-a), (uint16_t)(y0+b), color);             //1                
		LCD_DrawPoint((uint16_t)(x0-a), (uint16_t)(y0-b), color);             //2             
		LCD_DrawPoint((uint16_t)(x0+b), (uint16_t)(y0+a), color);             //4               
		LCD_DrawPoint((uint16_t)(x0+a), (uint16_t)(y0-b), color);             //5
		LCD_DrawPoint((uint16_t)(x0+a), (uint16_t)(y0+b), color);             //6 
		LCD_DrawPoint((uint16_t)(x0-b), (uint16_t)(y0+a), color);             //7
		a++;
		if((a*a+b*b)>(r*r))//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
      函数说明：显示单个字符
******************************************************************************/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加模式
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//叠加模式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}

/******************************************************************************
      函数说明：显示字符串
******************************************************************************/
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}

/******************************************************************************
      函数说明：显示数字
******************************************************************************/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}

/******************************************************************************
      函数说明：显示整数变量
******************************************************************************/
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 

/******************************************************************************
      函数说明：显示两位小数变量
******************************************************************************/
void LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp,sizex;
	uint16_t num1;
	sizex=sizey/2;
    // 修复警告：强制转换为 uint16_t
	num1 = (uint16_t)(num * 100);
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}

/******************************************************************************
      函数说明：显示有符号整数变量（支持负数）
******************************************************************************/
void LCD_ShowIntNumSigned(int32_t num, uint16_t x, uint16_t y, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{         
    uint8_t t, temp;
    uint8_t enshow = 0;
    uint8_t sizex = sizey / 2;
    uint32_t abs_num;
    
    // 处理负数
    if (num < 0)
    {
        LCD_ShowChar(x, y, '-', fc, bc, sizey, 0);
        x += sizex;
        len--;  // 负号占一位
        abs_num = (uint32_t)(-num);
    }
    else
    {
        abs_num = (uint32_t)num;
    }
    
    for (t = 0; t < len; t++)
    {
        temp = (abs_num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
                continue;
            }
            else enshow = 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

/******************************************************************************
      函数说明：显示有符号小数变量（支持负数，支持更多小数位）
******************************************************************************/
void LCD_ShowFloatNumSigned(float num, uint16_t x, uint16_t y, uint8_t int_len, uint8_t dec_len, uint16_t fc, uint16_t bc, uint8_t sizey)
{         
    uint8_t t, temp;
    uint8_t sizex = sizey / 2;
    float abs_num;
    uint32_t int_part, dec_part;
    
    // 处理负数
    if (num < 0)
    {
        LCD_ShowChar(x, y, '-', fc, bc, sizey, 0);
        x += sizex;
        int_len--;  // 负号占一位
        abs_num = -num;
    }
    else
    {
        abs_num = num;
    }
    
    // 计算整数部分和小数部分
    int_part = (uint32_t)abs_num;
    dec_part = (uint32_t)((abs_num - int_part) * mypow(10, dec_len));
    
    // 显示整数部分
    for (t = 0; t < int_len; t++)
    {
        temp = (int_part / mypow(10, int_len - t - 1)) % 10;
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
    
    // 显示小数点
    x += int_len * sizex;
    LCD_ShowChar(x, y, '.', fc, bc, sizey, 0);
    x += sizex;
    
    // 显示小数部分
    for (t = 0; t < dec_len; t++)
    {
        temp = (dec_part / mypow(10, dec_len - t - 1)) % 10;
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}
/******************************************************************************
      函数说明：显示十六进制数（大写字母）
******************************************************************************/
void LCD_ShowHexNum(uint32_t num, uint16_t x, uint16_t y, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{         
    uint8_t t, temp;
    uint8_t sizex = sizey / 2;
    
    for (t = 0; t < len; t++)
    {
        temp = (num >> ((len - t - 1) * 4)) & 0x0F;
        if (temp < 10)
        {
            LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);  // 0-9
        }
        else
        {
            LCD_ShowChar(x + t * sizex, y, temp + 55, fc, bc, sizey, 0);  // A-F
        }
    }
}
/******************************************************************************
      函数说明：显示十六进制数（带0x前缀）
******************************************************************************/
void LCD_ShowHexNumWithPrefix(uint32_t num, uint16_t x, uint16_t y, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{         
    uint8_t sizex = sizey / 2;
    
    // 显示"0x"前缀
    LCD_ShowChar(x, y, '0', fc, bc, sizey, 0);
    LCD_ShowChar(x + sizex, y, 'x', fc, bc, sizey, 0);
    
    // 显示十六进制数
    LCD_ShowHexNum(num, x + 2 * sizex, y, len, fc, bc, sizey);
}
/******************************************************************************
      函数说明：显示二进制数
******************************************************************************/
void LCD_ShowBinNum(uint32_t num, uint16_t x, uint16_t y, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{         
    uint8_t t, temp;
    uint8_t sizex = sizey / 2;
    
    for (t = 0; t < len; t++)
    {
        temp = (num >> ((len - t - 1) * 1)) & 0x01;
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);  // '0'或'1'
    }
}
/******************************************************************************
      函数说明：显示二进制数（带0b前缀）
******************************************************************************/
void LCD_ShowBinNumWithPrefix(uint32_t num, uint16_t x, uint16_t y, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{         
    uint8_t sizex = sizey / 2;
    
    // 显示"0b"前缀
    LCD_ShowChar(x, y, '0', fc, bc, sizey, 0);
    LCD_ShowChar(x + sizex, y, 'b', fc, bc, sizey, 0);
    
    // 显示二进制数
    LCD_ShowBinNum(num, x + 2 * sizex, y, len, fc, bc, sizey);
}



/******************************************************************************
      函数说明：显示图片
******************************************************************************/

void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[])
{
    LCD_Address_Set(x, y, x + length - 1, y + width - 1);
    
    // 直接把整个图片数组交给 DMA
    // 注意：pic 数组的大小必须是 length * width * 2
    LCD_Write_Buffer((uint8_t *)pic, (uint32_t)length * width * 2);
}
