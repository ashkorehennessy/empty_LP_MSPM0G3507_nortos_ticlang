#include "OLED.h"
#include "OLED_Font.h"

#define OLED_ADDR   0x3C

// PA30 SDA PA29 SCL  OLED   ok
void OLED_I2C_Init(void)
{

}

//向OLED寄存器地址写一个byte的数据
//int I2C_WriteByte(uint8_t addr,uint8_t data)
//{
//	uint8_t buff[2] = {0};
//	buff[0] = addr;
//	buff[1] = data;
//	// 等待I2C空闲状态
//	while (!(DL_I2C_getControllerStatus(I2C_OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
//
//	// 将数据包发送到控制器 , 此函数将自动发送Start + Stop
//	DL_I2C_startControllerTransfer(I2C_OLED_INST, OLED_ADDR,DL_I2C_CONTROLLER_DIRECTION_TX, 2);
//
//	DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, &buff[0], 2);
//	// 轮询直到控制器写入所有字节
//	while(DL_I2C_getControllerStatus(I2C_OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
//
//    /* Trap if there was an error */
//    if (DL_I2C_getControllerStatus(I2C_OLED_INST) & DL_I2C_CONTROLLER_STATUS_ERROR)
//	{
//        /* LED will remain high if there is an error */
//		__BKPT(0);
//    }
//    return 0;
//}



//写指令
void WriteCmd(unsigned char I2C_Command)
{
	I2C_WriteByte(0x00,I2C_Command);
}

//写数据
void WriteData(unsigned char I2C_Data)
{
	I2C_WriteByte(0x40,I2C_Data);
}

//厂家初始化代码
void OLED_Init(void)
{
    OLED_I2C_Init();
	delay_cycles(16000000);
	WriteCmd(0xAE); //display off
	WriteCmd(0x20); //Set Memory Addressing Mode
	WriteCmd(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0); //Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8); //Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //áá?èμ÷?ú 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel

}

//设置光标起始坐标（x,y）
void OLED_SetPos(unsigned char x,unsigned char y)
{
	WriteCmd(0xb0+y);
	WriteCmd( (x & 0xf0) >> 4 | 0x10 );
	WriteCmd( (x & 0x0f) | 0x01 );
}

//填充整个屏幕
void OLED_Fill(unsigned char Fill_Data)
{
	unsigned char m,n;

	for(m=0;m<8;m++)
	{
			WriteCmd(0xb0+m);
			WriteCmd(0x00);
			WriteCmd(0x10);

			for(n=0;n<128;n++)
			{
					WriteData(Fill_Data);
			}
	}
}

//清屏
void OLED_CLS(void)
{
	OLED_Fill(0x00);
}

//将OLED从休眠中唤醒
void OLED_ON(void)
{
	WriteCmd(0xAF);
	WriteCmd(0x8D);
	WriteCmd(0x14);
}

//让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
void OLED_OFF(void)
{
	WriteCmd(0xAE);
	WriteCmd(0x8D);
	WriteCmd(0x10);
}

void OLED_ShowStr(unsigned char x,unsigned char y,unsigned char ch[],unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;

	switch(TextSize)
	{
		case 1:
		{
				while(ch[j] != '\0')
				{
						c = ch[j] - 32;
						if(x>126)
						{
								x = 0;
								y++;
						}

						OLED_SetPos(x,y);

						for(i=0;i<6;i++)
						{
								WriteData(F6x8[c][i]);
						}
						x+=6;
						j++;
				}
		}
		break;

		case 2:
		{
				while(ch[j] != '\0')
				{
						c = ch[j] - 32;

						if(x>120)
						{
								x = 0;
								y++;
						}

						OLED_SetPos(x,y);

						for(i=0;i<8;i++)
						{
								WriteData(F8X16[c*16+i]);
						}

						OLED_SetPos(x,y+1);

						for(i=0;i<8;i++)
						{
								WriteData(F8X16[c*16+i+8]);
						}
						x+=8;
						j++;
				}
		}
		break;
	}
}

void OLED_ShowCN(unsigned char x,unsigned char y,unsigned char N)
{
	unsigned char i = 0;
	unsigned char addr = 32*N;

	OLED_SetPos(x,y);

	for(i=0;i<16;i++)
	{
			WriteData(F16X16[addr]);
			addr += 1;
	}

	OLED_SetPos(x,y+1);

	for(i=0;i<16;i++)
	{
			WriteData(F16X16[addr]);
			addr += 1;
	}
}

void OLED_ShowBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	unsigned char x,y;
	unsigned int j = 0;

	if(y1 % 8 == 0)
	{
			y = y1 / 8;
	}
	else
	{
			y = y1 / 8+1;
	}

	for(y=y0;y<y1;y++)
	{
			OLED_SetPos(x0,y);

			for(x=x0;x<x1;x++)
			{
					WriteData(BMP1[j++]);
			}
	}
}



