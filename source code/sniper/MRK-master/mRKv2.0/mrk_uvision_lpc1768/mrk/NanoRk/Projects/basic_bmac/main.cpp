#define pi 3.1415926
// positions for nets and home
#define X_net_a 5
#define Y_net_a -60
#define X_net_b 5
#define Y_net_b 85
#define X_assist_a 33
#define Y_assist_a -30
#define X_pass_a 53
#define Y_pass_a -30
#define X_assist_b -10
#define Y_assist_b 23
#define X_pass_b -40
#define Y_pass_b 28
#define X_home_a1 60
#define Y_home_a1 25
#define X_home_b1 80
#define Y_home_b1 -5
#define X_home_a2 -70
#define Y_home_a2 0
#define X_home_b2 -50
#define Y_home_b2 -25
// calibration orientations for team A&B 
#define A_orientation -10
#define B_orientation 170
#define ir_thresh 90
#define search_tol 900
#define defense_tol 110
#define angel_tol 3
//state code
#define INIT 1
#define SEARCH 2
#define APPROACH_NET 3
#define SHOOT 4
#define PAUSE 5
#define LOCALIZATION_A 6
#define APPROACH_HOME 7
#define LOCALIZATION_D 8
#define DEFENSE 9
#define PASS 10
#define ASSIST 11
#define WAIT 12
#define TXPASS 13
#define TXASSIST 14
//tx code
#define Hold 0
#define Release 1
unsigned int blob[12];
double theta;
double gamma;
double alpha;
double beta;
char state;
double XBOT;
double YBOT;
int previous = 0;
int count=0;
int lcount=0;
int rcount=0;
int dcount=0;
int ballCount=0;
int dist;
double net_scaler;
double net_dist;
int degree;
int x_init;
int y_init;
int id=4;  //identification of bots, 1/2 are snipers of team A, 2/3 are snipers of team B
char id_char;
int ad0;
int ad1;
int actFlag=99; //action flag is to prevent infinite state loop when processing package with same contents
int deFlag=0;  //defense flag, 0 for defense, 1 for back home and stay
int approachFlag;
int passFlag;  //1 for passing, 0 for non-passing, as well as pure attacker
int assistFlag; // 1 for assisting, 0 for non-assisting, as well as pure defender
int X_home;
int Y_home;
int X_net;
int Y_net;
int X_assist;
int Y_assist;
int X_pass;
int Y_pass;
int net;
int net_d;
int orientation;
int goal_A=0;
int goal_B=0;
int inited=0;
#include "position.h"
#include "wiicamera.h"
#include "goto.h"
#include "m3pi.h"
#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <string.h>
#include <math.h>
#include "mbed.h"
#include "basic_rf.h"
#include "bmac.h"
#include "adc_driver.h"
// Only require MAC address for address decode 
//#define MAC_ADDR	0x0001

m3pi m3pi(p23,p9,p10);
//Serial pc(USBTX, USBRX);
nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();

char tx_buf[RF_MAX_PAYLOAD_SIZE];
char rx_buf[RF_MAX_PAYLOAD_SIZE];

void assign(int id){
	if(id==1){
		id_char='A';
	X_home=X_home_a1;
	Y_home=Y_home_a1;
	X_net=X_net_a;
	Y_net=Y_net_a;
	X_assist=X_assist_a;
	Y_assist=Y_assist_a;
		X_pass=X_pass_a;
		Y_pass=Y_pass_a;
	orientation=A_orientation;
		approachFlag=0;
	  passFlag=1;
		assistFlag=0;
   }
   else if(id==2){
		 id_char='A';
	X_home=X_home_a2;
	Y_home=Y_home_a2;
	X_net=X_net_a;
	Y_net=Y_net_a;
  X_assist=X_assist_a;
	Y_assist=Y_assist_a;
		 	X_pass=X_pass_a;
		Y_pass=Y_pass_a;
	orientation=A_orientation;
	approachFlag=0;
		  passFlag=0;
		assistFlag=1;
   }
  else if(id==3){
		id_char='B';
	X_home=X_home_b1;
	Y_home=Y_home_b1;
	X_net=X_net_b;
	Y_net=Y_net_b;
		 X_assist=X_assist_b;
	Y_assist=Y_assist_b;
			X_pass=X_pass_b;
		Y_pass=Y_pass_b;
	orientation=B_orientation;
	approachFlag=1;
	passFlag=1;
	assistFlag=0;
  }
  else if(id==4){
		id_char='B';
	X_home=X_home_b2;
	Y_home=Y_home_b2;
	X_net=X_net_b;
	Y_net=Y_net_b;
		 X_assist=X_assist_b;
	Y_assist=Y_assist_b;
			X_pass=X_pass_b;
		Y_pass=Y_pass_b;
	orientation=B_orientation;
	approachFlag=1;
		 passFlag=0;
		assistFlag=1;
  }
}

void shoot(){
	
		m3pi.forward(1);
    wait_ms(250);
    m3pi.stop();
    wait_ms(200);
	  m3pi.backward(0.6);
    wait_ms(500);
    m3pi.stop();
    wait_ms(20);
}
void pass(){
	
		m3pi.forward(1);
    wait_ms(200);
    m3pi.stop();
    wait_ms(100);
	  m3pi.backward(0.6);
    wait_ms(400);
    m3pi.stop();
    wait_ms(20);
	
}

void stop(){     
	 m3pi.stop();
	 wait(5);
}

void rotate(int deg){
		if(deg<0){
		m3pi.left(0.2);
		wait_ms(1);
	}
	else{
		m3pi.right(0.2);
		wait_ms(1);
	}
}

void forward(int dist, int flag){  //flag=0 for attack, 1 for back home
	if(flag==0){
	int time=(int)(8*dist);
	m3pi.forward(0.4);
	wait_ms(time);
	m3pi.stop();
	wait_ms(1);
  }
  else if(flag==1){
	int time=(int)(11*dist);
	m3pi.forward(0.6);
	wait_ms(time);
	m3pi.stop();
	wait_ms(1);
  }
	else if(flag==2){
	if(id==2){
	int time=(int)(13*dist);
	m3pi.forward(0.55);
	wait_ms(time);
	m3pi.stop();
	wait_ms(1);
	}
	else if(id==4){
		int time=(int)(10*dist);
	m3pi.forward(0.55);
	wait_ms(time);
	m3pi.stop();
	wait_ms(1);
	}
  }
}

void go_forward(){     //exclusive for defense move
	m3pi.forward(0.5);
	wait_ms(300);
	m3pi.stop();
	wait(1);
	m3pi.backward(0.5);
	wait_ms(300);
	m3pi.stop();
	wait_ms(10);
}



void led_attacker(){
	nrk_led_set(RED_LED);
	nrk_led_set(GREEN_LED);
	nrk_led_clr(BLUE_LED);
	nrk_led_clr(ORANGE_LED);
}

void led_defender(){
	nrk_led_clr(RED_LED);
	nrk_led_clr(GREEN_LED);
	nrk_led_set(BLUE_LED);
	nrk_led_set(ORANGE_LED);
}
void led_passer(){
	nrk_led_clr(RED_LED);
	nrk_led_clr(GREEN_LED);
	nrk_led_clr(BLUE_LED);
	nrk_led_set(ORANGE_LED);
}
void led_assistant(){
	nrk_led_set(RED_LED);
	nrk_led_clr(GREEN_LED);
	nrk_led_clr(BLUE_LED);
	nrk_led_clr(ORANGE_LED);
}
void led_set(){
	nrk_led_set(RED_LED);
	nrk_led_set(GREEN_LED);
	nrk_led_set(BLUE_LED);
	nrk_led_set(ORANGE_LED);
}
void led_clr(){
	nrk_led_clr(RED_LED);
	nrk_led_clr(GREEN_LED);
	nrk_led_clr(BLUE_LED);
	nrk_led_clr(ORANGE_LED);
}


int main(void)

  {
//		m3pi.forward(0.5);
//		wait(1);
			nrk_setup_ports();
		printf("nrk ready\n");
			nrk_init();
			printf("nrk init ready\n");
		   init_adc();
			printf("adc ready\n");
		      clock_init();
			printf("clock ready\n");
		      cam_init();	
			printf("cam ready\n");
	  //pc.baud(9600);
	    assign(id);
			bmac_task_config();
			nrk_create_taskset();
	/*	  if(id%2) state=SEARCH;
		  else if(assistFlag==0)
				state=DEFENSE;
			else if(assistFlag==1)
				state=LOCALIZATION_A;*/
			nrk_start();
		  
			return 0;

	}
	
	

void rx_task ()
{
  uint8_t i, len, rssi;
  int8_t val;
	char *local_rx_buf;
  nrk_time_t check_period;
//  printf ("rx_task PID=%d\r\n", nrk_get_pid ());

  // init bmac on channel 24 
  bmac_init (24);
	

   bmac_set_cca_thresh(DEFAULT_BMAC_CCA); 


  // This sets the next RX buffer.
  // This can be called at anytime before releaseing the packet
  // if you wish to do a zero-copy buffer switch
  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

  while (1) {
    // Wait until an RX packet is received
    val = bmac_wait_until_rx_pkt ();
		
    // Get the RX packet 

    local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
    //if( bmac_rx_pkt_is_encrypted()==1 ) nrk_kprintf( PSTR( "Packet Encrypted\r\n" ));
 //   printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
    //for (i = 0; i < len; i++)
    //  printf ("%c", rx_buf[i]);
 //   printf ("]\r\n");

			if(rx_buf[2]=='0'&&actFlag!=0&&state==SEARCH){   //0 for hold, actFlag prevents the infinite state loop
				m3pi.stop();
				wait_ms(10);
					led_set();
				m3pi.stop();
				wait_ms(50);
				led_clr();
				m3pi.backward(0.5);             //backoff a little so to avoid collision and jam
				wait_ms(350);
				m3pi.stop();
				wait_ms(10);
				
		/*		m3pi.right(0.4);
				wait_ms(200);
					m3pi.stop();
				wait_ms(10);*/
				state=DEFENSE;                  //changeable, defense or localization_d, it's a problem
				
		//		state=LOCALIZATION_D;  
				actFlag=0;
		//		deFlag=0;
			}
			else if(rx_buf[2]=='1'&&actFlag!=1&&rx_buf[0]!=id_char){ //1 for release, now go search again
				if(id%2) state=SEARCH;
				else state=DEFENSE;
				actFlag=1;
				led_set();
				m3pi.stop();
				wait_ms(100);
				led_clr();
			}
			else if(rx_buf[2]=='2'&&actFlag!=2){  //2 for goal, now come back home
	//			printf("back home!!!!\n");
				m3pi.stop();
				wait_ms(10);
				m3pi.backward(0.5);
				wait_ms(500);
        m3pi.stop();
				wait_ms(10);
				state=WAIT; 
				actFlag=2;
				deFlag=1;
				led_set();
				m3pi.stop();
				wait_ms(100);
				led_clr();
	//			goal_A=rx_buf[3];
	//			goal_B=rx_buf[4];
			}
	    
			else if(rx_buf[2]=='3'&&actFlag!=3&&rx_buf[0]==id_char){   //3 for assistant on position, try passing the ball now
			//	X_mate=atoi(&rx_buf[3]);
		//		Y_mate=atoi(&rx_buf[4]);
		//		state=PASS;
				actFlag=3;
					led_set();
				m3pi.stop();
				wait_ms(100);
				led_clr();
			}
			else if(rx_buf[2]=='4'&&actFlag!=4&&rx_buf[0]==id_char){   //4 for assistant on position, try passing the ball now
	      led_set();
				m3pi.stop();
				wait_ms(1);
				led_clr();
			
				state=SEARCH;
				actFlag=4;
			}
			
			else if(rx_buf[2]=='5'&&inited==0){
			  goal_A=rx_buf[3];
				goal_B=rx_buf[4];
				inited=1;
				state=INIT;
			}
  
   
		// Release the RX buffer so future packets can arrive 
    bmac_rx_pkt_release ();
		
		// this is necessary
    nrk_wait_until_next_period ();

  }

}


uint8_t ctr_cnt[4];

void transmit(int info){
	 uint8_t j, i, val, len, cnt;
  int8_t v;
  nrk_sig_t tx_done_signal;
  nrk_sig_mask_t ret;
  nrk_time_t r_period;
	
	// printf("tx_task PID=%d\r\n", nrk_get_pid ());

 
  while (!bmac_started ())
    nrk_wait_until_next_period ();
  

 
  tx_done_signal = bmac_get_tx_done_signal ();
  nrk_signal_register (tx_done_signal);

  ctr_cnt[0]=0; ctr_cnt[1]=0; ctr_cnt[2]=0; ctr_cnt[3]=0;
  cnt = 0;
	   sprintf(tx_buf,"%c%d%d\n",id_char,id,info);
	   bmac_auto_ack_disable();  
     val=bmac_tx_pkt(tx_buf, strlen(tx_buf));
		 if(val==NRK_OK) cnt++;
}

void tx_task ()
{
/*  uint8_t j, i, val, len, cnt;
  int8_t v;
  nrk_sig_t tx_done_signal;
  nrk_sig_mask_t ret;
  nrk_time_t r_period;
	
	// printf("tx_task PID=%d\r\n", nrk_get_pid ());

 
  while (!bmac_started ())
    nrk_wait_until_next_period ();
  

 
  tx_done_signal = bmac_get_tx_done_signal ();
  nrk_signal_register (tx_done_signal);

  ctr_cnt[0]=0; ctr_cnt[1]=0; ctr_cnt[2]=0; ctr_cnt[3]=0;
  cnt = 0;*/
	
while (1) {
    // Build a TX packet
	
 /* 
read_camera(blob);
	track(blob);
	   int x_current=XBOT;
         int y_current=YBOT;
	dist=(int)sqrt((double)(x_current-X_net)*(double)(x_current-X_net)+(double)(y_current-Y_net)*(double)(y_current-Y_net));
      
	printf("XBOT: %d, YBOT: %d, theta: %d, dist: %d, Blob: %d,%d,%d,%d,%d,%d,%d,%d count: %d\n",(int)XBOT,(int)YBOT, (int)(theta*180/pi),dist,blob[0],blob[1],blob[3],blob[4],blob[6],blob[7],blob[9],blob[10], cnt);
*/


	
//------------------------------------ROUTINE----------------------------------------//
switch(state)    {
      
default:
   state=WAIT;
   break;
//-----------------------------------INITIALIZATION-------------------------------//        
case INIT:
 //   deFlag=0;        //reset defense flag
    actFlag=99;
   
    if(id==1) {
				if(goal_A>goal_B){
					state=SEARCH;
					passFlag=0;
				}
				else if(goal_A==goal_B){
					state=SEARCH;
					passFlag=1;
				}
				else if(goal_A<goal_B){				
					state=SEARCH;
					passFlag=0;
				}
			}
			
		else if(id==3){
			if(goal_A<goal_B){
					state=SEARCH;
					passFlag=0;
				}
				else if(goal_A==goal_B){
					state=SEARCH;
					passFlag=1;
				}
				else if(goal_A>goal_B){				
					state=SEARCH;
					passFlag=0;
				}
		}
    else {
			if(id==2){
				if(goal_A>goal_B){
					state=DEFENSE;
					assistFlag=0;
				}
				else if(goal_A==goal_B){
					state=LOCALIZATION_A;
					assistFlag=1;
				}
				else if(goal_A<goal_B){				
					state=SEARCH;
					assistFlag=0;
				}
			}
			else if(id==4){
				if(goal_B>goal_A){
					state=DEFENSE;
					assistFlag=0;
				}
				else if(goal_B==goal_A){
					state=LOCALIZATION_A;
					assistFlag=1;
				}
        else if(goal_B<goal_A){				
					state=SEARCH;
					passFlag=0;
				}
			}			
		}
      break;
//----------------------------------SEARCH THE BALL-------------------------------//            
case SEARCH:
	  led_attacker();
//		sprintf(tx_buf,"%c%d search %d\n", id_char,id,cnt);
       set_adc_chan(0);
       ad0=get_adc_val();
      set_adc_chan(1);
       ad1=get_adc_val();
  //    printf("AD0: %d, AD1: %d\n",ad0,ad1);
      if(ad0<search_tol||ad1<search_tol){
          if(abs(ad0-ad1)<90){
              m3pi.forward(0.2);
              wait_ms(5);
          
          }
          else if (ad0>ad1){
              m3pi.left(0.15);
              wait_ms(5);
          }
          else if(ad1>ad0){
              m3pi.right(0.15);
              wait_ms(5);
          }        
      }
      if(ad0<ir_thresh&&ad1<ir_thresh){
				   transmit(Hold);
          m3pi.forward(0.3);
          wait_ms(300);

				if(passFlag==1) 
					state=PASS;
				else
          state=LOCALIZATION_A;
         
      }
      count=0;
      break;
      
//-------------------------------ATTACK LOCALIZATION-------------------------------//            
case LOCALIZATION_A:
	  led_attacker();
	//	sprintf(tx_buf," localization attack %d\n", cnt);
  //        printf("localization!\n");
      read_camera(blob);
      track(blob);
//printf("XBOT: %d, YBOT: %d, theta: %d, Blob: %d,%d,%d,%d,%d,%d,%d,%d count: %d\n",(int)XBOT,(int)YBOT, (int)theta,blob[0],blob[1],blob[3],blob[4],blob[6],blob[7],blob[9],blob[10], cnt);
//sprintf(tx_buf,"XBOT: %d, YBOT: %d, theta: %d, Blob: %d,%d,%d,%d,%d,%d,%d,%d count: %d\n",(int)XBOT,(int)YBOT, (int)theta,blob[0],blob[1],blob[3],blob[4],blob[6],blob[7],blob[9],blob[10], cnt);
  if(approachFlag==0){    
      int deg=orientation-(int)(theta*180/pi);
      if(abs(deg)>angel_tol){
          rotate(deg);
          
      }
      else{
           int x_current=XBOT;
           int y_current=YBOT;
           dist=(int)sqrt((double)(x_current-X_net)*(double)(x_current-X_net)+(double)(y_current-Y_net)*(double)(y_current-Y_net));
				if(assistFlag==0)
            state=APPROACH_NET;
				else
					state=ASSIST;
      }
  }
  else if(approachFlag==1){
      int deg=orientation-(int)(theta*180/pi);
     // if((int)(theta*180/pi)<0||(int)(theta*180/pi)>174||(int)(theta*180/pi)==0){
		  if((int)(theta*180/pi)<-10||(int)(theta*180/pi)>174){
          m3pi.left(0.2);
          wait_ms(1);
    //      m3pi.stop();
  //    wait_ms(1);
      }
      else{
          if(abs(deg)>angel_tol){
          rotate(deg);
          
      }
      else{
				if(passFlag==1){
					m3pi.stop();
					wait_ms(10);
				}
				else{
           int x_current=XBOT;
           int y_current=YBOT;
           dist=(int)sqrt((double)(x_current-X_net)*(double)(x_current-X_net)+(double)(y_current-Y_net)*(double)(y_current-Y_net));
           if(assistFlag==0)
             state=APPROACH_NET;
				else
					state=ASSIST;
				}
      }
  }
  }
      break;
//---------------------------------DEFENSE LOCALIZATION-------------------------------//            
case LOCALIZATION_D:
	  led_set();
//		sprintf(tx_buf," localization defense %d\n", cnt);
  //        printf("localization!\n");
      read_camera(blob);
      track(blob);
  
      int deg_d=orientation-(int)(theta*180/pi);
  if(approachFlag==0){    
      int deg=orientation-(int)(theta*180/pi);
      if(abs(deg)>angel_tol){
          rotate(deg);
          
      }
      else{
           int x_current=XBOT;
           int y_current=YBOT;
           dist=(int)sqrt((double)(x_current-X_home)*(double)(x_current-X_home)+(double)(y_current-Y_home)*(double)(y_current-Y_home));
           state=APPROACH_HOME;
      }
  }
  else if(approachFlag==1){
      int deg=orientation-(int)(theta*180/pi);
    //  if((int)(theta*180/pi)<0||(int)(theta*180/pi)>174||(int)(theta*180/pi)==0){
		  if((int)(theta*180/pi)<-10||(int)(theta*180/pi)>174){
          m3pi.left(0.2);
          wait_ms(1);
       //   m3pi.stop();
       //  wait_ms(1);
      }
      else{
          if(abs(deg)>angel_tol){
          rotate(deg);
          
      }
      else{
           int x_current=XBOT;
           int y_current=YBOT;
           dist=(int)sqrt((double)(x_current-X_home)*(double)(x_current-X_home)+(double)(y_current-Y_home)*(double)(y_current-Y_home));
           state=APPROACH_HOME;
      }
  }
  }
      break;
//------------------------------APPROACHING NET-------------------------------//            
case APPROACH_NET:
	led_attacker();
//	sprintf(tx_buf," approach net %d\n", cnt);
    
      read_camera(blob);
      track_deg(blob);
    
      
      if(approachFlag==0) net=approach_net_A(X_net,Y_net);
      else                net=approach_net_B(X_net,Y_net);
      
          if(net==1){
               degree=180-(int)(beta*180/pi);
          }
          else if(net==2){
               degree=(int)(beta*180/pi);
          }
          else if(net==3){
               degree=-(180-(int)(beta*180/pi));
          }
          else if(net==4){
               degree=-(int)(beta*180/pi);
          }
          
  //     printf("net:%d\n", net);
    if(approachFlag==0){
      if(abs(orientation + degree-(int)(theta*180/pi))>angel_tol){
          
   //       printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
          forward(dist,0);
          state=SHOOT;
      }        
      }
  else{
      if(orientation+degree>180){
 
          if(abs(orientation + degree-360-(int)(theta*180/pi))>angel_tol){
              
    //      printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-180-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
          forward(dist,0);
          state=SHOOT;
      }    

      }
     else{
        if(abs(orientation + degree-(int)(theta*180/pi))>angel_tol){
              
  //        printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-180-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
          forward(dist,0);
          state=SHOOT;
      }    
     }
   }
      break;
//------------------------------APPROACH HOME-------------------------------//            
case APPROACH_HOME:
	led_set();
//	sprintf(tx_buf," approach home dist: %d,  cnt:%d\n", dist,cnt);
      //printf("approach!\n");
      read_camera(blob);
      track_deg(blob);
 
   
      if(approachFlag==0)
          net_d=approach_net_A(X_home,Y_home);
            else
                  net_d=approach_net_B(X_home,Y_home);
      if(net_d==1){
               degree=180-(int)(beta*180/pi);
          }
          else if(net_d==2){
               degree=(int)(beta*180/pi);
          }
          else if(net_d==3){
               degree=-(180-(int)(beta*180/pi));
          }
          else if(net_d==4){
               degree=-(int)(beta*180/pi);
          }
          
  //     printf("net_d:%d\n", net_d);
  if(approachFlag==0){
      if(abs(orientation + degree-(int)(theta*180/pi))>5){
   //       printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
          forward(dist,1);
				 state=WAIT;
    
      }        
  }
  else{
      if(orientation+degree>180){
 
          if(abs(orientation + degree-360-(int)(theta*180/pi))>5){
  //        printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-180-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
          forward(dist,1);
				 state=WAIT;
     
      }    
 
  }
  else{
          if(abs(orientation + degree-(int)(theta*180/pi))>5){
  //        printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-180-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
          forward(dist,1);
				 state=WAIT;
       
      }    
  }
}
      break;
//-----------------------------DEFENSE-------------------------------//    
case DEFENSE:
	led_defender();
       set_adc_chan(0);
       ad0=get_adc_val();
      set_adc_chan(1);
       ad1=get_adc_val();
  //    printf("AD0: %d, AD1: %d\n",ad0,ad1);
      if(ad0<600||ad1<600){
          if(abs(ad0-ad1)<80&&(ad0<defense_tol||ad1<defense_tol)){
              go_forward();
          
          }
          else if (ad0>ad1){
              m3pi.left(0.15);
              wait_ms(5);
          }
          else if(ad0<ad1){
          m3pi.right(0.15);
              wait_ms(5);
          }        
      }
      else{
          m3pi.stop();
          wait_ms(1);
      }
      break;
//---------------------------PASS--------------------------------//
case PASS:
	led_passer();
	//	sprintf(tx_buf," approach net %d\n", cnt);
    //  printf("approach!\n");
      read_camera(blob);
      track_deg(blob);
     transmit(4);
      
      if(approachFlag==0) net=approach_net_A(X_pass,Y_pass);
      else                net=approach_net_B(X_pass,Y_pass);
      
          if(net==1){
               degree=180-(int)(beta*180/pi);
          }
          else if(net==2){
               degree=(int)(beta*180/pi);
          }
          else if(net==3){
               degree=-(180-(int)(beta*180/pi));
          }
          else if(net==4){
               degree=-(int)(beta*180/pi);
          }
          
  //     printf("net:%d\n", net);
    if(approachFlag==0){
      if(abs(orientation + degree-(int)(theta*180/pi))>angel_tol){
          
   //       printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
      
        pass();
				m3pi.stop();
				wait_ms(10);
			
        state=TXPASS;
      }        
      }
  else{
      if(orientation+degree>180){
 
          if(abs(orientation + degree-360-(int)(theta*180/pi))>angel_tol){
              
    //      printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-180-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
       
        pass();
				m3pi.stop();
				wait_ms(10);
					
        state=TXPASS;
      }    

      }
     else{
        if(abs(orientation + degree-(int)(theta*180/pi))>angel_tol){
              
  //        printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-180-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
 
				  pass();
				m3pi.stop();
				wait_ms(10);
				
        state=TXPASS;
      }    
     }
   }
      break;
//-------------------------TXPASS--------------------------------//
case TXPASS:
	led_passer();
  transmit(4);
   count++;
   if(count>2000) {
		 state=SEARCH;
		 passFlag=0;
		 count=0;
	 }
   break;

//-------------------------TXASSIST------------------------------//
case TXASSIST:
  led_assistant();
if(count<2000)
  transmit(3);
else{
	count=0;
	state=WAIT;
}
  count++;
   break;
//--------------------------ASSIST-------------------------------//
case ASSIST:
	 led_assistant();

      read_camera(blob);
      track_deg(blob);
     
      
      if(approachFlag==0) net=approach_net_A(X_assist,Y_assist);
      else                net=approach_net_B(X_assist,Y_assist);
      
          if(net==1){
               degree=180-(int)(beta*180/pi);
          }
          else if(net==2){
               degree=(int)(beta*180/pi);
          }
          else if(net==3){
               degree=-(180-(int)(beta*180/pi));
          }
          else if(net==4){
               degree=-(int)(beta*180/pi);
          }
          
  //     printf("net:%d\n", net);
    if(approachFlag==0){
      if(abs(orientation + degree-(int)(theta*180/pi))>angel_tol){
          
   //       printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
          forward(dist,2);//go a little deeper
			
				m3pi.stop();
				wait_ms(10);
					m3pi.left(0.4);
				wait_ms(250);
				m3pi.stop();
				wait_ms(10);
				state=TXASSIST;
					assistFlag=0;
      }        
      }
  else{
      if(orientation+degree>180){
 
          if(abs(orientation + degree-360-(int)(theta*180/pi))>angel_tol){
              
    //      printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-180-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
          forward(dist,2);
		
				m3pi.stop();
				wait_ms(10);
					m3pi.left(0.4);
				wait_ms(250);
				m3pi.stop();
				wait_ms(10);
				state=TXASSIST;
				assistFlag=0;
      }    

      }
     else{
        if(abs(orientation + degree-(int)(theta*180/pi))>angel_tol){
              
  //        printf("degree:%d,  v:%d, theta:%d\n",degree,abs(orientation + degree-180-(int)(theta*180/pi)),(int)(theta*180/pi));
          rotate(degree);
      }
      else{
          forward(dist,2);
		
				m3pi.stop();
				wait_ms(10);
			  m3pi.left(0.4);
				wait_ms(250);
				m3pi.stop();
				wait_ms(10);
				state=TXASSIST;
				assistFlag=0;
      }    
     }
   }
      break;
//---------------------------WAIT--------------------------------//
case WAIT:
   m3pi.stop();
   wait_ms(1);
   break;

//---------------------------SHOOT-------------------------------//            
case SHOOT:
	led_attacker();
  transmit(Release);
   shoot();
    state=SEARCH;
    break;
//----------------------------PAUSE-------------------------------//    
case PAUSE:
	led_clr();
  stop();
    break;
      
  }	
	
 //		 else printf("NO ack or Reserve Violated! \r\n");

    nrk_wait_until_next_period ();
  }

}

void nrk_create_taskset ()
{
  RX_TASK.task = rx_task;
  nrk_task_set_stk( &RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 4;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
  RX_TASK.period.secs = 0;
  RX_TASK.period.nano_secs = 5*NANOS_PER_MS;
  RX_TASK.cpu_reserve.secs = 1;
  RX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);

  TX_TASK.task = tx_task;
  nrk_task_set_stk( &TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 3;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 0;
  TX_TASK.period.nano_secs = 5*NANOS_PER_MS;
  TX_TASK.cpu_reserve.secs = 1;
  TX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);
}