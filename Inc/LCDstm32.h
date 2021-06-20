/* ----------------------------------------------------------------------------
library name :lcd.h -by this library you can work with at91sam family without
any long statment .
this library is provide by 1NAFAR (at91sam7x645@gmail.com) for at91sam series 
you can edit , remove & change all parts af that .also you can download other 
library for other atmel & nxp products (AT91-LPC series)from WWW.IRANMICRO.IR 
you can access to descriptions ADC  Resistors Control in PMM 3 magazine . for 
more information please visit : www.iranmicro.ir/forum/forumdisplay.php?f=221
*/
#include <stm32f0xx.h>
#ifndef lcd_INCLUDED_
#define lcd_INCLUDED_

#ifdef   LCD_PORT_A
#define  PIO_SetOutput	   GPIOA ->BSRR
#define  PIO_ClearOutput   GPIOA ->BRR
#define  PIO_CfgOutput  GPIOA ->MODER
#endif
//#ifdef LCD_PORT_B
 //#define   PIO_enable	   *AT91C_PIOB_PER
//#define  PIO_SetOutput	       *AT91C_PIOB_SODR  
//#define  PIO_ClearOutput	   *AT91C_PIOB_CODR
//#define  PIO_CfgOutput  *AT91C_PIOB_OER 
//#endif
#define F_CPU	48000000
#define clcd_minDelay()	cwait(F_CPU/8000)
#define clcd_Delay()	cwait(F_CPU/800)
void lcd_init(void);
void lcd_putchar(char ch);
void lcd_command(char cmd);
void lcd_gotoxy(char x, char y);
void lcd_clear(void);
void lcd_clear_line(char y);
void lcd_shift_left(char n);
void lcd_shift_right(char n);
void lcd_puts(int num);
void lcd_putsf(char* data);
void lcd_define_char(const char *pc,char char_code);
//int lcd_busy();
//=========================================================
void cwait (volatile int t) {
  for (;t>0; t--);
}
// ============================================================================
void set_lcd_db(char val)
{
	PIO_ClearOutput |=((1<<LCD_DB4)|(1<<LCD_DB5)|(1<<LCD_DB6)|(1<<LCD_DB7));
	if (val & 0x80)
		PIO_SetOutput |=(1<<LCD_DB7)	;
	if (val & 0x40)
		PIO_SetOutput |=(1<<LCD_DB6);
	if (val & 0x20)
                PIO_SetOutput |=(1<<LCD_DB5);
	if (val & 0x10)
                PIO_SetOutput |=(1<<LCD_DB4);
}
// ============================================================================
void lcd_putchar(char ch)
{
	PIO_SetOutput |=(1<<LCD_RS);
	set_lcd_db(ch);
	PIO_SetOutput |=(1<<LCD_E);
	clcd_minDelay();
	PIO_ClearOutput |=(1<<LCD_E);
	clcd_minDelay();
	set_lcd_db(ch<<4);
	PIO_SetOutput |=(1<<LCD_E);
	clcd_minDelay();
        PIO_ClearOutput |=(1<<LCD_E);
	clcd_Delay();
}
// ============================================================================
void lcd_command(char cmd)	//Sends Command to LCD
{
	PIO_ClearOutput |=(1<<LCD_RS);
	set_lcd_db(cmd);
	PIO_SetOutput |=(1<<LCD_E);
	clcd_minDelay();
	PIO_ClearOutput |=(1<<LCD_E);
	clcd_minDelay();
	set_lcd_db(cmd<<4);
	PIO_SetOutput |=(1<<LCD_E);
	clcd_minDelay();
	PIO_ClearOutput |=(1<<LCD_E);
	clcd_Delay();
}
// ============================================================================
void lcd_init()
{
	RCC ->AHBENR |= 0x20000;
	PIO_CfgOutput |= ((1<<(2*LCD_RW))|(1<<(2*LCD_RS))|(1<<(2*LCD_E))|(1<<(2*LCD_DB4))|(1<<(2*LCD_DB5))|(1<<(2*LCD_DB6))|(1<<(2*LCD_DB7)));
        
  
	clcd_Delay();
	set_lcd_db(0);
	set_lcd_db((1<<5)|(1<<4));
        PIO_SetOutput |=(1<<LCD_E);
	clcd_minDelay();
        PIO_ClearOutput |=(1<<LCD_E);
	clcd_Delay();
	set_lcd_db((1<<5)|(1<<4));
	PIO_SetOutput |=(1<<LCD_E);
	clcd_minDelay();
	PIO_ClearOutput |=(1<<LCD_E);
	clcd_Delay();
	set_lcd_db(1<<5);
	PIO_SetOutput |=(1<<LCD_E);
	clcd_minDelay();
	PIO_ClearOutput |=(1<<LCD_E);
	clcd_Delay();
	lcd_command(0x28);
	lcd_command(0x0c);
	clcd_Delay();
}
//*******************************************************

/*
int lcd_busy(){
int flag = 1;

  PIO_ClearOutput = (1<<LCD_RS);
  *AT91C_PIOA_ODR = (1<<LCD_DB7);
  PIO_SetOutput = (1<<LCD_RW);
  
while(flag==1)
{
PIO_SetOutput = (1<<LCD_E);
flag = (AT91C_BASE_PIOA->PIO_PDSR & (1<<LCD_DB7));
  PIO_ClearOutput = (1<<LCD_E);

  
  PIO_SetOutput = (1<<LCD_E);
   PIO_ClearOutput = (1<<LCD_E);
 
}
  *AT91C_PIOA_OER = (1<<LCD_DB7);
   PIO_ClearOutput = (1<<LCD_RW);
return 0;
}
*/
//*******************************************************
void lcd_gotoxy(char y, char x)	//Cursor to X Y position
{
	char DDRAMAddr;
x=x-1;
	switch(y)
	{
		case 1: DDRAMAddr = 0x00+x; break;
		case 2: DDRAMAddr = 0x40+x; break;
		case 3: DDRAMAddr = 0x14+x; break;
		case 4: DDRAMAddr = 0x54+x; break;
		default: DDRAMAddr = 0x00+x;
	}
	lcd_command(1<<7 | DDRAMAddr);
}
// ============================================================================
void lcd_define_char(const char *pc,char char_code)
{
	char a , i;
	a = ((char_code<<3)|0x40) & 0xff;
	for (i = 0; i < 8 ;i++)
	{
		lcd_command(a++);
		clcd_Delay();
		lcd_putchar(pc[i]);
		clcd_Delay();
	}
}
// ============================================================================
void lcd_clear(void)				//Clears LCD
{
	lcd_command(0x01);
        lcd_command(0x02);
	clcd_Delay();
	clcd_Delay();
}
//============================================================
void lcd_shift_left(char n)	//Scrol n of characters Right
{
	char i;
	for (i = 0 ; i < n ; i++)
	{
		lcd_command(0x1E);
		clcd_Delay();
	}
}
//========================================================
void lcd_shift_right(char n)	//Scrol n of characters Left
{
	char i;
	for (i = 0 ; i < n ; i++)
	{
		lcd_command(0x18);
		clcd_Delay();
	}
}
//============================================================================
 void lcd_puts(int num)
{
   int i,j;
   int p,f=0;
   char ch[5];
 for(i=0;i<5;i++)
   { 
     p=1;
     for(j=4-i;j>0;j--)
       p = p*10;
     ch[i] = (num/p);
	 if (num>=p && !f){
	  f=1; 
	 }
     num =num - ch[i] *p ;
	 ch[i] = ch[i] +48;
     if(f) lcd_putchar(ch[i]);
   }
}
// ============================================================================
void lcd_putsf(char* data)	//Outputs string to LCD
{
	char *p;
	p = data;
	if (!p)
		return;
	for(; *p ; )
	{
		lcd_putchar(*p);
		p++;
	}
}
//=============================================================
void Display_off(void)
{
	lcd_command(0x08);
clcd_Delay();
}
//=============================================================
void Display_on(void)
{
	lcd_command(0x0C);
	clcd_Delay();
}
//=============================================================
void cursor_off(void)
{
	lcd_command(0x0C);
	clcd_Delay();
}
//=============================================================
void cursor_on(void)
{
	lcd_command(0x0E);
	clcd_Delay();
}
//=============================================================
void cursor_blink(void)
{
	lcd_command(0x0F);
	clcd_Delay();
}
#endif
