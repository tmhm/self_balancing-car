/********************************************************

  SA0接地
  SCL B2引脚启用I2C0 SCL功能  本程序是模拟实现
  SDA B3引脚启用I2C0 SDA功能  本程序是模拟实现
  3.3V
  GND


2013-03-6 测试通过

三轴陀螺仪模块上直接接地了
三轴加速度模块上可以跳线选择
*********************************************************/	
#include <stdio.h>
#include "include.h"
#include "mma8451.h"
#include "arm_math.h"


  

//IO口初始化
void IO_Init(void)
{
    /* 打开各个端口的时钟源 */
    SIM_SCGC5 |=  SIM_SCGC5_PORTB_MASK; //| 
//              SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
//    PORTA_PCR14=PORT_PCR_MUX(1);//A14引脚设置为GPIO模式
//    PORTA_PCR15=PORT_PCR_MUX(1);//A15引脚设置为GPIO模式
//    PORTA_PCR16=PORT_PCR_MUX(1);//A16引脚设置为GPIO模式
//    PORTA_PCR17=PORT_PCR_MUX(1);//A17引脚设置为GPIO模式	    
     //设置PORTA pin14,pin15为输出方向;pin16,pin17为输入方向
//    GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(14)|GPIO_PIN(15)|GPIO_PIN(16)|GPIO_PIN(17));    
    
    //D2口GPIO功能，PE启用拉电阻，PS上拉电阻
    PORTB_PCR2 = PORT_PCR_MUX(1); //FOR IIC0  SCL
    PORTB_PCR3 = PORT_PCR_MUX(1);//|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;  //SDA
    
    //设置PORTB pin2,pin3为输出方向
    GPIOB_PDDR=GPIO_PDDR_PDD(GPIO_PIN(2)|GPIO_PIN(3));

}


//端口位定义，B2 SCL;B3 SDA
#define SDA     GPIOB_PDIR&0X0008                         //IO口输入 SDA
#define SDA0()  GPIOB_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(3))	//IO口输出低电平
#define SDA1()  GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(3))	//IO口输出高电平  
#define SCL0()  GPIOB_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(2))	//IO口输出低电平
#define SCL1()  GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2))	//IO口输出高电平
#define DIR_OUT()  GPIOB_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(3))         //输出方向
#define DIR_IN()   GPIOB_PDDR&=~GPIO_PDDR_PDD(GPIO_PIN(3))     //输入方向

//SA0必须接地

void iicdelay()
{   
	      asm("nop");asm("nop");asm("nop"); asm("nop");       //2013.4.26xw测试延时不能再短了
               asm("nop");          //2013.4.27加了一次延时，貌似经常掉数
           //   asm("nop");asm("nop");asm("nop");
              asm("nop");
              asm("nop");   //长沙比赛临时加
              asm("nop"); 
}

//内部数据定义
unsigned char IIC_ad_main; //器件从地址	    
unsigned char IIC_ad_sub;  //器件子地址	   
unsigned char *IIC_buf;    //发送|接收数据缓冲区	    
unsigned char IIC_num;     //发送|接收数据个数	     

#define ack 1      //主应答
#define no_ack 0   //从应答	 

//nop指令个数定义   
//#define iicdelay() {asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}  
void IIC_start(void){
	SCL0();
	SDA1();
	iicdelay();
	SCL1();
	iicdelay();
	SDA0();
	iicdelay();
	SCL0();
}
//************************************************
//送停止位 SDA=0->1
void IIC_stop(void){
	SCL0();
	iicdelay();
	SDA0();
	iicdelay();
	SCL1();
	iicdelay();
	SDA1();
	iicdelay();
	SCL0();
}
//************************************************
//主应答(包含ack:SDA=0和no_ack:SDA=0)
void IIC_ack_main(unsigned char ack_main){
	SCL0();
	if(ack_main) SDA0(); //ack主应答
	else SDA1(); //no_ack无需应答
	iicdelay();
	SCL1();
	iicdelay();
	SCL0();
}
//*************************************************
//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
void send_ch(unsigned char c){
	unsigned char i;
	for(i=0;i<8;i++){
		SCL0();
		if((c<<i) & 0x80)SDA1(); //判断发送位
		else SDA0();
		iicdelay();
		SCL1();
		iicdelay();
		SCL0();
	}
	iicdelay();
	SDA1(); //发送完8bit，释放总线准备接收应答位
	iicdelay();
	SCL1();
	iicdelay(); //sda上数据即是从应答位              
	SCL0(); //不考虑从应答位|但要控制好时序
}
//**************************************************
//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|IIC_ack_main()使用
//return: uchar型1字节
unsigned char read_ch(void){
unsigned char i;
unsigned char c;
	c=0;
	SCL0();
	iicdelay();
	SDA1(); //置数据线为输入方式
	DIR_IN();
	for(i=0;i<8;i++){
		iicdelay();
		SCL0(); //置时钟线为低，准备接收数据位
		iicdelay();
		SCL1(); //置时钟线为高，使数据线上数据有效
		iicdelay();
		c<<=1;
		if(SDA) c+=1; //读数据位，将接收的数据存c
	}
	SCL0();
	DIR_OUT();
	return c;
}
//***************************************************
//向无子地址器件发送单字节数据
void send_to_ch(unsigned char ad_main,unsigned char c){
	IIC_start();
	send_ch(ad_main); //发送器件地址
	send_ch(c); //发送数据c
	IIC_stop();
}
//***************************************************
//向有子地址器件发送多字节数据
void send_to_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num){
	unsigned char i;
	IIC_start();
	send_ch(ad_main); //发送器件地址
	send_ch(ad_sub); //发送器件子地址
	for(i=0;i<num;i++){
		send_ch(*buf); //发送数据*buf
		buf++;
	}
	IIC_stop();
}
//***************************************************
//从无子地址器件读单字节数据
//function:器件地址，所读数据存在接收缓冲区当前字节
void read_from_ch(unsigned char ad_main,unsigned char *buf){
	IIC_start();
	send_ch(ad_main); //发送器件地址
	*buf=read_ch();
	IIC_ack_main(no_ack); //无需应答<no_ack=0>
	IIC_stop();
}
//***************************************************
//从有子地址器件读多个字节数据
//function:
void read_from_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num){
	unsigned char i;
	IIC_start();
	send_ch(ad_main);
	send_ch(ad_sub);
	for(i=0;i<num-1;i++){
		*buf=read_ch();
		IIC_ack_main(ack); //主应答<ack=1>
		buf++;
	}
	*buf=read_ch();
	buf++; //本次指针调整无意义，目的是操作后buf指向下一地址
	IIC_ack_main(no_ack); //无需应答<no_ack=0>
	IIC_stop();
}




unsigned char MMA845x_readch(unsigned char address)
{
unsigned char ret = 100;
	IIC_start();		//启动
	send_ch(MMA845x_IIC_ADDRESS);	//写入设备ID及写信号
	send_ch(address);	//X地址
	IIC_start();		//重新发送开始
	send_ch(MMA845x_IIC_ADDRESS+1);	//写入设备ID及读信
	ret = read_ch();	//读取一字节
	IIC_stop();

	return ret;
}

//写入
void MMA845x_writecha(unsigned char address, unsigned char thedata)
{
	IIC_start();		//启动
	send_ch(MMA845x_IIC_ADDRESS);	//写入设备ID及写信号
	send_ch(address);	//X地址
	send_ch(thedata);	//写入设备ID及读信
	IIC_stop();
}

//初始化
//初始化为指定模式
void MMA845_init(void)
{	
        IO_Init();	//将模拟用的两个引脚B2、B3初始化
	MMA845x_writecha(CTRL_REG1,ASLP_RATE_20MS+DATA_RATE_5MS);	  //貌似不稳定，多加了一次iicdelay（）。2013.4.27
	iicdelay(); iicdelay(); iicdelay();// iicdelay();// iicdelay(); iicdelay(); 
        //iicdelay(); //iicdelay(); //iicdelay(); //iicdelay();// iicdelay();// iicdelay(); 
	MMA845x_writecha(XYZ_DATA_CFG_REG, FULL_SCALE_2G); //2G
	iicdelay(); iicdelay(); iicdelay();// iicdelay(); iicdelay(); iicdelay();
       // iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); 
	MMA845x_writecha(CTRL_REG1, (ACTIVE_MASK+ASLP_RATE_20MS+DATA_RATE_5MS)&(~FREAD_MASK)); //激活状态   14bit
	iicdelay();iicdelay(); iicdelay(); //iicdelay(); iicdelay(); iicdelay();
        //iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); 
}


void mmadelay()
{
   int i;
   int j;
	for(i=0;i<300;i++)
	  for(j=0;j<10;j++)
	      asm("nop");
}
void longdelay()
{
   int i;
   int j;
	for(i=0;i<3000;i++)
	  for(j=0;j<1000;j++)
	      asm("nop");
}

unsigned char  MMAValue[2] = {0x00,0x00};

///////////////////////////////////////////////////////////moni_mma8451
extern  s16  VOLTAGE_GRAVITY;
//extern   u8 MMA8451_flag;
void Get_mma8451_z(void)
{	   
        u8  i,z=0;
     //   u8  x=0;
        s32 sum=0;
        s16 wz=0;
    //    s16 wx=0;
        s16 result_8451_z=0;

        u8  v=0;
   	v= MMA845x_readch(WHO_AM_I_REG);
	if(v == MMA8451Q_ID)    //||(v == MMA8452Q_ID)||(v == MMA8453Q_ID)) 
	{
        //  gpio_set (PORTC , 16, 0);
	 // printf("\ninit OK!");    
	}
	else
	{
        //  gpio_set (PORTC , 16, 1);
	 // printf("\ninit Failed!");  	
	}
           //读取重力信息 
       /*     x = MMA845x_readch(OUT_X_MSB_REG);
            wx = ((MMA845x_readch(OUT_X_LSB_REG))|x<<8);
            
              if(x>0x7f) //补码求出加速度原始对应数值
              {			          
                wx=(~(wx>>2) + 1)&0X3FFF; //移位取反加一再去掉无效字符            
                Acc_Xvalue  = -wx;
              }
              else
              {
                wx=(wx>>2)&0X3FFF; 	//移位再去掉无效字符 
                Acc_Xvalue  = +wx;
              }  */
          //  y = MMA845x_readch(OUT_Y_MSB_REG);
          //  wy = ((MMA845x_readch(OUT_Y_LSB_REG))|y<<8);
        
         for(i=2;i>0;i--)           //4次读加速度计共用210us的周期
         {   
             z = MMA845x_readch(OUT_Z_MSB_REG); 
             wz = ((MMA845x_readch(OUT_Z_LSB_REG))|z<<8);  
        
             if(z>0x7f) //补码求出加速度原始对应数值
              {			          
                wz=(~(wz>>2) + 1)&0X3FFF; //移位取反加一再去掉无效字符            
                result_8451_z=-wz;
              }
              else
              {
                wz=(wz>>2)&0X3FFF; 	//移位再去掉无效字符 
                result_8451_z=+wz;
              }  
             sum += result_8451_z;
         } 
         VOLTAGE_GRAVITY =(s16)( sum/2);      
}
