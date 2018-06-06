/******************************************************************
*Dosya          :lcd.c
*Yazar          :Ayhan KORKMAZ - AyhanKorkmaz.Net
*Iletisim           :info@ayhankorkmaz.net
*Versiyon       :V1.0
*******************************************************************/
 
#include "lcd.h"

 unsigned char sayac2,adres=64,s1=0;
 
 
 
 #define RS		GPIO_Pin_8
 #define En		GPIO_Pin_3
 

#define  D4		GPIO_Pin_4
#define	 D5		GPIO_Pin_5
#define  D6		GPIO_Pin_6
#define	 D7		GPIO_Pin_7 
 

#define GPIO_Pin_8 4
#define GPIO_Pin_3 5

#define GPIO_Pin_4 10
#define GPIO_Pin_5 11
#define GPIO_Pin_6 12
#define GPIO_Pin_7 13

#define lcd_data_port gpioPortE
#define lcd_komut_port gpioPortF
 
/*****************************************************************************
Kullanim  : Eklemek istediginiz özel karakterleri ekleyebiliriniz
            Bunun için LCD karakter olusturma programlarini kulanabilirsiniz
******************************************************************************/
 unsigned char karakter_[8][8]=
{
  /* TR Karakterler */
{ 0,14,16,16,17,14, 4, 0},//ç
{ 0, 0,12, 4, 4, 4,14, 0},//I
{10, 0,14,17,17,17,14, 0},//ö
{ 0,15,16,14, 1,30, 4, 0},//$
{10, 0,17,17,17,17,14, 0},//ü
/* özel isaretler */
{2, 6,14,30,14, 6,  2, 0},//<
{ 0, 4, 4, 4, 4, 4, 4, 0},//|
{ 0, 16, 8, 4, 2, 1, 0,0} //\//
};
/********************************************************
Fonksiyon : delay
Amac      : Gecikme Fonksiyonudur.
Kullanim  : delay(0x0000FFFF);
*********************************************************/
 void delay(unsigned long delay)
{
 while(delay--);
}
/********************************************************
Fonksiyon : lcd_komut
Amac      : LCD Çalismasi için gerekli komutlari verir
Kullanim  : lcd_komut(temizle);
*********************************************************/
void lcd_komut(char komut)
{
		GPIO_WriteBit(lcd_komut_port,RS,0); //GPIOC->BRR =  0x00000001;            //RS=0

		GPIO_WriteBit(lcd_data_port,D7,0);
		GPIO_WriteBit(lcd_data_port,D6,0);
		GPIO_WriteBit(lcd_data_port,D5,0);
		GPIO_WriteBit(lcd_data_port,D4,0);
		if(komut & 0x80)
				GPIO_WriteBit(lcd_data_port,D7,1);
		if(komut & 0x40)
				GPIO_WriteBit(lcd_data_port,D6,1);
		if(komut & 0x20)
				GPIO_WriteBit(lcd_data_port,D5,1);
		if(komut & 0x10)
				GPIO_WriteBit(lcd_data_port,D4,1);			//GPIOC->ODR |= (komut & 0x000000F0);  

		GPIO_WriteBit(lcd_komut_port,En,1);			//GPIOC->ODR |= 0x00000002;            //E=1
    lcd_gecikme();
    GPIO_WriteBit(lcd_komut_port,En,0);					//GPIOC->BRR = 0x00000002;         //E=0
		lcd_gecikme();
		
		GPIO_WriteBit(lcd_data_port,D7,0);
		GPIO_WriteBit(lcd_data_port,D6,0);
		GPIO_WriteBit(lcd_data_port,D5,0);
		GPIO_WriteBit(lcd_data_port,D4,0);
 
    lcd_gecikme();
		
    if(komut & 0x08)
				GPIO_WriteBit(lcd_data_port,D7,1);
		if(komut & 0x04)
				GPIO_WriteBit(lcd_data_port,D6,1);
		if(komut & 0x02)
				GPIO_WriteBit(lcd_data_port,D5,1);
		if(komut & 0x01)
				GPIO_WriteBit(lcd_data_port,D4,1);
		
		GPIO_WriteBit(lcd_komut_port,En,1);			//GPIOC->ODR |= 0x00000002;            //E=1
    lcd_gecikme();
    GPIO_WriteBit(lcd_komut_port,En,0);					//GPIOC->BRR = 0x00000002;         //E=0
		lcd_gecikme();
		
		GPIO_WriteBit(lcd_data_port,D7,0);
		GPIO_WriteBit(lcd_data_port,D6,0);
		GPIO_WriteBit(lcd_data_port,D5,0);
		GPIO_WriteBit(lcd_data_port,D4,0);
		
		lcd_gecikme();
}
/********************************************************
Fonksiyon : lcd_karakter_yaz
Amac      : LCD Ekrana bir karakter basar
Kullanim  : lcd_karakter_yaz('A');
*********************************************************/
void lcd_karakter_yaz(char veri)
{
	char veri1;
	veri1=veri;
switch (veri) {
case 'Ç' : veri=0x00; break;
case 'I' : veri=0x01; break;
case 'Ö' : veri=0x02; break;
case 'S' : veri=0x03; break;
case 'Ü' : veri=0x04; break;
 
case 'ç' : veri=0x00; break;
case 'i' : veri=0x01; break;
case 'ö' : veri=0x02; break;
case 's' : veri=0x03; break;
case 'ü' : veri=0x04; break;
 
default : break;
}
 
		GPIO_WriteBit(lcd_komut_port,RS,1); //GPIOC->BRR =  0x00000001;            //RS=0


		GPIO_WriteBit(lcd_data_port,D7,0);
		GPIO_WriteBit(lcd_data_port,D6,0);
		GPIO_WriteBit(lcd_data_port,D5,0);
		GPIO_WriteBit(lcd_data_port,D4,0);

		if(veri & 0x80)
				GPIO_WriteBit(lcd_data_port,D7,1);
		if(veri & 0x40)
				GPIO_WriteBit(lcd_data_port,D6,1);
		if(veri & 0x20)
				GPIO_WriteBit(lcd_data_port,D5,1);
		if(veri & 0x10)
				GPIO_WriteBit(lcd_data_port,D4,1);
		
		GPIO_WriteBit(lcd_komut_port,En,1);//GPIOC->ODR |= 0x00000002;            //E=1
    lcd_gecikme();
    GPIO_WriteBit(lcd_komut_port,En,0);					//GPIOC->BRR = 0x00000002;         //E=0
		lcd_gecikme();
		
		
		
		GPIO_WriteBit(lcd_data_port,D7,0);
		GPIO_WriteBit(lcd_data_port,D6,0);
		GPIO_WriteBit(lcd_data_port,D5,0);
		GPIO_WriteBit(lcd_data_port,D4,0);
 
    lcd_gecikme();
		
		

		
    if(veri & 0x08)
				GPIO_WriteBit(lcd_data_port,D7,1);
		if(veri & 0x04)
				GPIO_WriteBit(lcd_data_port,D6,1);
		if(veri & 0x02)
				GPIO_WriteBit(lcd_data_port,D5,1);
		if(veri & 0x01)
				GPIO_WriteBit(lcd_data_port,D4,1);
		
		GPIO_WriteBit(lcd_komut_port,En,1);			//GPIOC->ODR |= 0x00000002;            //E=1
    lcd_gecikme();
    GPIO_WriteBit(lcd_komut_port,En,0);					//GPIOC->BRR = 0x00000002;         //E=0
		lcd_gecikme();
		
		
		GPIO_WriteBit(lcd_data_port,D7,0);
		GPIO_WriteBit(lcd_data_port,D6,0);
		GPIO_WriteBit(lcd_data_port,D5,0);
		GPIO_WriteBit(lcd_data_port,D4,0);
		
    lcd_gecikme();
}
/********************************************************
Fonksiyon : lcd_yazi_yaz
Amac      : LCD Ekrana yazi yazar
Kullanim  : lcd_yazi_yaz("AyhanKorkmaz.Net");
*********************************************************/
void lcd_yazi_yaz(char *veri)
{
 
    while(*veri)
    {
        lcd_gecikme();
        lcd_karakter_yaz(*veri++);
 
    }
          delay(0x0000FFFF);
}
/**************************************************************
Fonksiyon : lcd_git_xy
Amac      : LCD ekranin hangi bölmesine yazilacagini ayarlariz
Kullanim  : lcd_git_xy(2,1); 2. satir 1. Sütun
***************************************************************/
void lcd_git_xy(unsigned char satir,unsigned char sutun)
{
    if(satir==1)
    {
        lcd_komut(0x00000080 + (sutun-1));                  //1.satir 1.sütun için cgram adresi 0x80 dir
    }
    else if(satir==2)
    {
        lcd_komut(0x000000C0 +(sutun-1));                   //2.satir 1.sütun için chram adresi 0x80+0x40=0xC0 dir
    }
}
/********************************************************
Fonksiyon : lcd_hazirla
Amac      : LCD Çalismasi için gerekli ilk ayarlar
Kullanim  : lcd_hazirla();
*********************************************************/
void lcd_hazirla(void)
{
	

	
    delay(0x0000FFFF);
    delay(0x0000FFFF);

    delay(0x0000FFFF);
 
    //RCC_DeInit();


 
    lcd_komut(zorunlu);
          delay(0x0000FFFF);
    lcd_komut(ikisatir4bit5x8);
          delay(0x0000FFFF);
    lcd_komut(imlecsagakay);
          delay(0x0000FFFF);
    lcd_komut(dayansonig);
          delay(0x0000FFFF);
    lcd_komut(dayansonyok);
          delay(0x0000FFFF);
    lcd_komut(temizle);
          delay(0x0000FFFF);
    lcd_gecikme();
 
		lcd_temizle();
 
    for(sayac2=0;sayac2<=7;sayac2++){  // Türkçe karakterler tanitiliyor
    lcd_komut(adres);
    for(s1=0;s1<=7;s1++){
        lcd_karakter_yaz(karakter_[sayac2][s1]);
                     }
        adres=adres+8;
                 }
        lcd_temizle();
         delay(0x0000FFFF);
 
}
/********************************************************
Fonksiyon : lcd_temizle
Amac      : LCD ekran tamamen silinir
Kullanim  : lcd_temizle();
*********************************************************/
void lcd_temizle(void)
{
    lcd_komut(temizle);
    lcd_gecikme();
}
/********************************************************
Fonksiyon : lcd_gecikme
Amac      : LCD Çalismasi için gerekli gecikmedir
Kullanim  : lcd_gecikme();
*********************************************************/
void lcd_gecikme(void)
{
    unsigned long delay=0x00001100;
    while(delay--);
}

void lcd_init(void)
{

 CMU_ClockEnable(cmuClock_GPIO, true);
	
	GPIO_PinModeSet(lcd_data_port, 10, gpioModePushPull, 0);
  GPIO_PinOutClear(lcd_data_port, 10);
	GPIO_PinModeSet(lcd_data_port, 11, gpioModePushPull, 0);
  GPIO_PinOutClear(lcd_data_port, 11);
	GPIO_PinModeSet(lcd_data_port, 12, gpioModePushPull, 0);
  GPIO_PinOutClear(lcd_data_port, 12);
	GPIO_PinModeSet(lcd_data_port, 13, gpioModePushPull, 0);
  GPIO_PinOutClear(lcd_data_port, 13);
	
	GPIO_PinModeSet(lcd_komut_port, 3, gpioModePushPull, 0);
  GPIO_PinOutClear(lcd_komut_port, 3);
	GPIO_PinModeSet(lcd_komut_port, 4, gpioModePushPull, 0);
  GPIO_PinOutClear(lcd_komut_port, 4);
	GPIO_PinModeSet(lcd_komut_port, 5, gpioModePushPull, 0);
  GPIO_PinOutClear(lcd_komut_port, 5);
	
	

}

void GPIO_WriteBit(GPIO_Port_TypeDef port, uint32_t pins,bool enable)
{

		if(enable)
			GPIO_PinOutSet(port,pins);
		else
			GPIO_PinOutClear(port,pins);


}