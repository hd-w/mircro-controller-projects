
sbit OLED_DC =P3^5;//??/????
sbit OLED_SCL=P2^0;//?? D0(SCLK?
sbit OLED_SDIN=P3^7;//D1(MOSI) ??



sbit OLED_CS=P3^4;
sbit OLED_RST = P3^6;

#define SIZE 16
#define XLevelL		0x02
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64

#define  u8 unsigned char 
#define  u32 unsigned int 
#define OLED_CMD  0	//???
#define OLED_DATA 1	//???
#define OLED_MODE 0

#define OLED_CS_Clr() OLED_CS =0
#define OLED_CS_Set()  OLED_CS =1
#define OLED_RST_Clr() OLED_RST =0
#define OLED_RST_Set() OLED_RST = 1
#define OLED_DC_Clr() OLED_DC=0
#define OLED_DC_Set() OLED_DC=1
#define OLED_SCLK_Clr() OLED_SCL=0
#define OLED_SCLK_Set() OLED_SCL=1
#define OLED_SDIN_Clr() OLED_SDIN=0
#define OLED_SDIN_Set() OLED_SDIN=1;


void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2);
void OLED_ShowString(u8 x,u8 y, u8 *p);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void OLED_Set_Pos1(unsigned char x, unsigned char y);
void OLED_DrawPixel(u8 x0, u8 y0, u8 pixel,u8 BMP[]);
void OLED_DrawPixel_Direct(u8 x0, u8 y0, u8 pixel);
void OLED_DrawByAngle(float seita,u8 BMP[]);