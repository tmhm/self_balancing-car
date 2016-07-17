/********************************************************

  SA0�ӵ�
  SCL B2��������I2C0 SCL����  ��������ģ��ʵ��
  SDA B3��������I2C0 SDA����  ��������ģ��ʵ��
  3.3V
  GND


2013-03-6 ����ͨ��

����������ģ����ֱ�ӽӵ���
������ٶ�ģ���Ͽ�������ѡ��
*********************************************************/	
#include <stdio.h>
#include "include.h"
#include "mma8451.h"
#include "arm_math.h"


  

//IO�ڳ�ʼ��
void IO_Init(void)
{
    /* �򿪸����˿ڵ�ʱ��Դ */
    SIM_SCGC5 |=  SIM_SCGC5_PORTB_MASK; //| 
//              SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
//    PORTA_PCR14=PORT_PCR_MUX(1);//A14��������ΪGPIOģʽ
//    PORTA_PCR15=PORT_PCR_MUX(1);//A15��������ΪGPIOģʽ
//    PORTA_PCR16=PORT_PCR_MUX(1);//A16��������ΪGPIOģʽ
//    PORTA_PCR17=PORT_PCR_MUX(1);//A17��������ΪGPIOģʽ	    
     //����PORTA pin14,pin15Ϊ�������;pin16,pin17Ϊ���뷽��
//    GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(14)|GPIO_PIN(15)|GPIO_PIN(16)|GPIO_PIN(17));    
    
    //D2��GPIO���ܣ�PE���������裬PS��������
    PORTB_PCR2 = PORT_PCR_MUX(1); //FOR IIC0  SCL
    PORTB_PCR3 = PORT_PCR_MUX(1);//|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;  //SDA
    
    //����PORTB pin2,pin3Ϊ�������
    GPIOB_PDDR=GPIO_PDDR_PDD(GPIO_PIN(2)|GPIO_PIN(3));

}


//�˿�λ���壬B2 SCL;B3 SDA
#define SDA     GPIOB_PDIR&0X0008                         //IO������ SDA
#define SDA0()  GPIOB_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(3))	//IO������͵�ƽ
#define SDA1()  GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(3))	//IO������ߵ�ƽ  
#define SCL0()  GPIOB_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(2))	//IO������͵�ƽ
#define SCL1()  GPIOB_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2))	//IO������ߵ�ƽ
#define DIR_OUT()  GPIOB_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(3))         //�������
#define DIR_IN()   GPIOB_PDDR&=~GPIO_PDDR_PDD(GPIO_PIN(3))     //���뷽��

//SA0����ӵ�

void iicdelay()
{   
	      asm("nop");asm("nop");asm("nop"); asm("nop");       //2013.4.26xw������ʱ�����ٶ���
               asm("nop");          //2013.4.27����һ����ʱ��ò�ƾ�������
           //   asm("nop");asm("nop");asm("nop");
              asm("nop");
              asm("nop");   //��ɳ������ʱ��
              asm("nop"); 
}

//�ڲ����ݶ���
unsigned char IIC_ad_main; //�����ӵ�ַ	    
unsigned char IIC_ad_sub;  //�����ӵ�ַ	   
unsigned char *IIC_buf;    //����|�������ݻ�����	    
unsigned char IIC_num;     //����|�������ݸ���	     

#define ack 1      //��Ӧ��
#define no_ack 0   //��Ӧ��	 

//nopָ���������   
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
//��ֹͣλ SDA=0->1
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
//��Ӧ��(����ack:SDA=0��no_ack:SDA=0)
void IIC_ack_main(unsigned char ack_main){
	SCL0();
	if(ack_main) SDA0(); //ack��Ӧ��
	else SDA1(); //no_ack����Ӧ��
	iicdelay();
	SCL1();
	iicdelay();
	SCL0();
}
//*************************************************
//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
void send_ch(unsigned char c){
	unsigned char i;
	for(i=0;i<8;i++){
		SCL0();
		if((c<<i) & 0x80)SDA1(); //�жϷ���λ
		else SDA0();
		iicdelay();
		SCL1();
		iicdelay();
		SCL0();
	}
	iicdelay();
	SDA1(); //������8bit���ͷ�����׼������Ӧ��λ
	iicdelay();
	SCL1();
	iicdelay(); //sda�����ݼ��Ǵ�Ӧ��λ              
	SCL0(); //�����Ǵ�Ӧ��λ|��Ҫ���ƺ�ʱ��
}
//**************************************************
//�ֽڽ��ճ���
//�����������������ݣ��˳���Ӧ���|��Ӧ����|IIC_ack_main()ʹ��
//return: uchar��1�ֽ�
unsigned char read_ch(void){
unsigned char i;
unsigned char c;
	c=0;
	SCL0();
	iicdelay();
	SDA1(); //��������Ϊ���뷽ʽ
	DIR_IN();
	for(i=0;i<8;i++){
		iicdelay();
		SCL0(); //��ʱ����Ϊ�ͣ�׼����������λ
		iicdelay();
		SCL1(); //��ʱ����Ϊ�ߣ�ʹ��������������Ч
		iicdelay();
		c<<=1;
		if(SDA) c+=1; //������λ�������յ����ݴ�c
	}
	SCL0();
	DIR_OUT();
	return c;
}
//***************************************************
//�����ӵ�ַ�������͵��ֽ�����
void send_to_ch(unsigned char ad_main,unsigned char c){
	IIC_start();
	send_ch(ad_main); //����������ַ
	send_ch(c); //��������c
	IIC_stop();
}
//***************************************************
//�����ӵ�ַ�������Ͷ��ֽ�����
void send_to_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num){
	unsigned char i;
	IIC_start();
	send_ch(ad_main); //����������ַ
	send_ch(ad_sub); //���������ӵ�ַ
	for(i=0;i<num;i++){
		send_ch(*buf); //��������*buf
		buf++;
	}
	IIC_stop();
}
//***************************************************
//�����ӵ�ַ���������ֽ�����
//function:������ַ���������ݴ��ڽ��ջ�������ǰ�ֽ�
void read_from_ch(unsigned char ad_main,unsigned char *buf){
	IIC_start();
	send_ch(ad_main); //����������ַ
	*buf=read_ch();
	IIC_ack_main(no_ack); //����Ӧ��<no_ack=0>
	IIC_stop();
}
//***************************************************
//�����ӵ�ַ����������ֽ�����
//function:
void read_from_nch(unsigned char ad_main,unsigned char ad_sub,unsigned char *buf,unsigned char num){
	unsigned char i;
	IIC_start();
	send_ch(ad_main);
	send_ch(ad_sub);
	for(i=0;i<num-1;i++){
		*buf=read_ch();
		IIC_ack_main(ack); //��Ӧ��<ack=1>
		buf++;
	}
	*buf=read_ch();
	buf++; //����ָ����������壬Ŀ���ǲ�����bufָ����һ��ַ
	IIC_ack_main(no_ack); //����Ӧ��<no_ack=0>
	IIC_stop();
}




unsigned char MMA845x_readch(unsigned char address)
{
unsigned char ret = 100;
	IIC_start();		//����
	send_ch(MMA845x_IIC_ADDRESS);	//д���豸ID��д�ź�
	send_ch(address);	//X��ַ
	IIC_start();		//���·��Ϳ�ʼ
	send_ch(MMA845x_IIC_ADDRESS+1);	//д���豸ID������
	ret = read_ch();	//��ȡһ�ֽ�
	IIC_stop();

	return ret;
}

//д��
void MMA845x_writecha(unsigned char address, unsigned char thedata)
{
	IIC_start();		//����
	send_ch(MMA845x_IIC_ADDRESS);	//д���豸ID��д�ź�
	send_ch(address);	//X��ַ
	send_ch(thedata);	//д���豸ID������
	IIC_stop();
}

//��ʼ��
//��ʼ��Ϊָ��ģʽ
void MMA845_init(void)
{	
        IO_Init();	//��ģ���õ���������B2��B3��ʼ��
	MMA845x_writecha(CTRL_REG1,ASLP_RATE_20MS+DATA_RATE_5MS);	  //ò�Ʋ��ȶ��������һ��iicdelay������2013.4.27
	iicdelay(); iicdelay(); iicdelay();// iicdelay();// iicdelay(); iicdelay(); 
        //iicdelay(); //iicdelay(); //iicdelay(); //iicdelay();// iicdelay();// iicdelay(); 
	MMA845x_writecha(XYZ_DATA_CFG_REG, FULL_SCALE_2G); //2G
	iicdelay(); iicdelay(); iicdelay();// iicdelay(); iicdelay(); iicdelay();
       // iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); iicdelay(); 
	MMA845x_writecha(CTRL_REG1, (ACTIVE_MASK+ASLP_RATE_20MS+DATA_RATE_5MS)&(~FREAD_MASK)); //����״̬   14bit
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
           //��ȡ������Ϣ 
       /*     x = MMA845x_readch(OUT_X_MSB_REG);
            wx = ((MMA845x_readch(OUT_X_LSB_REG))|x<<8);
            
              if(x>0x7f) //����������ٶ�ԭʼ��Ӧ��ֵ
              {			          
                wx=(~(wx>>2) + 1)&0X3FFF; //��λȡ����һ��ȥ����Ч�ַ�            
                Acc_Xvalue  = -wx;
              }
              else
              {
                wx=(wx>>2)&0X3FFF; 	//��λ��ȥ����Ч�ַ� 
                Acc_Xvalue  = +wx;
              }  */
          //  y = MMA845x_readch(OUT_Y_MSB_REG);
          //  wy = ((MMA845x_readch(OUT_Y_LSB_REG))|y<<8);
        
         for(i=2;i>0;i--)           //4�ζ����ٶȼƹ���210us������
         {   
             z = MMA845x_readch(OUT_Z_MSB_REG); 
             wz = ((MMA845x_readch(OUT_Z_LSB_REG))|z<<8);  
        
             if(z>0x7f) //����������ٶ�ԭʼ��Ӧ��ֵ
              {			          
                wz=(~(wz>>2) + 1)&0X3FFF; //��λȡ����һ��ȥ����Ч�ַ�            
                result_8451_z=-wz;
              }
              else
              {
                wz=(wz>>2)&0X3FFF; 	//��λ��ȥ����Ч�ַ� 
                result_8451_z=+wz;
              }  
             sum += result_8451_z;
         } 
         VOLTAGE_GRAVITY =(s16)( sum/2);      
}
