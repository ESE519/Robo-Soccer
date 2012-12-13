#include "mbed.h"

// Adapted from kako's source code: http://www.kako.com/neta/2008-009/2008-009.html
// i2c protocol details from - http://blog.makezine.com/archive/2008/11/hacking_the_wiimote_ir_ca.html
// wiring from - http://translate.google.com/translate?u=http://www.kako.com/neta/2007-001/2007-001.html&hl=en&ie=UTF-8&sl=ja&tl=en
// obviously mbed is 3.3v so no level translation is needed
// using built in i2c on pins 9/10
//
// PC GUI client here: http://code.google.com/p/wii-cam-blobtrack/
//
// Interfacing details here: http://www.bot-thoughts.com/2010/12/connecting-mbed-to-wiimote-ir-camera.html
//

//DigitalOut myled(LED1);
PwmOut servo(p24);
//Serial pc(USBTX, USBRX); // tx, rx
I2C i2c(p28, p27);        // sda, scl
const int addr = 0xB0;   // define the I2C Address of camera

void i2c_write2(int addr, char a, char b)
{
    char cmd[2];
    
    cmd[0] = a;
    cmd[1] = b;
    i2c.write(addr, cmd, 2);
    wait(0.07); // delay 70ms    
}

void clock_init()
{
    // set up ~20-25MHz clock on p21
	 
    LPC_PWM1->TCR = (1 << 1);               // Reset counter, disable PWM
    LPC_SC->PCLKSEL0 &= ~(0x3 << 12);  
    LPC_SC->PCLKSEL0 |= (1 << 12);          // Set peripheral clock divider to /1, i.e. system clock
    LPC_PWM1->MR0 = 4;                     // Match Register 0 is shared period counter for all PWM1
    LPC_PWM1->MR6 = 2;                      // Pin 21 is PWM output 6, so Match Register 6
    LPC_PWM1->LER |= 1;                     // Start updating at next period start
    LPC_PWM1->TCR = (1 << 0) || (1 << 3);   // Enable counter and PWM
}

void cam_init()
{
    // Init IR Camera sensor
    i2c_write2(addr, 0x30, 0x01);
    i2c_write2(addr, 0x30, 0x08);    
    i2c_write2(addr, 0x06, 0x90);
    i2c_write2(addr, 0x08, 0xC0);
    i2c_write2(addr, 0x1A, 0x40);
    i2c_write2(addr, 0x33, 0x33);
    wait(0.1);
 //   pc.baud(115200);
}

void read_camera(unsigned int ret[]) {

    char cmd[8];
    char buf[16];
    int Ix1,Iy1,Ix2,Iy2;
    int Ix3,Iy3,Ix4,Iy4;
    int s;
    int flag;
    //clock_init();
    
    // PC serial output    
    
    //pc.printf("Initializing camera...");

  
    //pc.printf("complete\n");
    
    // read I2C stuff
    
    //    myled = 1;
        // IR Sensor read
        cmd[0] = 0x36;
        i2c.write(addr, cmd, 1);
        i2c.read(addr, buf, 16); // read the 16-byte result

  //     myled = 0;
        
        Ix1 = buf[1];
        Iy1 = buf[2];
        s   = buf[3];
        Ix1 += (s & 0x30) <<4;
        Iy1 += (s & 0xC0) <<2;
        
        Ix2 = buf[4];
        Iy2 = buf[5];
        s   = buf[6];
        Ix2 += (s & 0x30) <<4;
        Iy2 += (s & 0xC0) <<2;
        
        Ix3 = buf[7];
        Iy3 = buf[8];
        s   = buf[9];
        Ix3 += (s & 0x30) <<4;
        Iy3 += (s & 0xC0) <<2;
        
        Ix4 = buf[10];
        Iy4 = buf[11];
        s   = buf[12];
        Ix4 += (s & 0x30) <<4;
        Iy4 += (s & 0xC0) <<2;
				  ret[0]=Ix1;
        ret[1]=Iy1;
        
        ret[3]=Ix2;
        ret[4]=Iy2;
        ret[6]=Ix3;
        ret[7]=Iy3;
				ret[9]=Ix4;
				ret[10]=Iy4;
   /*  if(Ix4==1023){
        			 
        ret[0]=Ix1;
        ret[1]=Iy1;
        
        ret[3]=Ix2;
        ret[4]=Iy2;
        ret[6]=Ix3;
        ret[7]=Iy3;
		 }
		 else if(Ix3==1023){
			  ret[0]=Ix1;
        ret[1]=Iy1;
        
        ret[3]=Ix2;
        ret[4]=Iy2;
        ret[6]=Ix4;
        ret[7]=Iy4;
		 }
		  else if(Ix2==1023){
			  ret[0]=Ix1;
        ret[1]=Iy1;
        
        ret[3]=Ix3;
        ret[4]=Iy3;
        ret[6]=Ix4;
        ret[7]=Iy4;
		 }
		  else if(Ix1==1023){
			  ret[0]=Ix2;
        ret[1]=Iy2;
        
        ret[3]=Ix3;
        ret[4]=Iy3;
        ret[6]=Ix4;
        ret[7]=Iy4;
		 }*/
    //    ret[9]=Ix4;
    //   ret[10]=Iy4;
        // print the coordinate data
    //    printf("Ix1: %4d, Iy1: %4d\n", Ix1, Iy1 );
    //    printf("Ix2: %4d, Iy2: %4d\n", Ix2, Iy2 );
        
        
        wait(.05);
    }


