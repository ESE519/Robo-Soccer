#define pi 3.1415926
#define net_scaler_max 0.3		// maximum net scaler value
#define net_tol 16
#define beta_tol 20
#define X_net 31
#define Y_net -45
#define X_home 9
#define Y_home 18
#define k 1
#define INIT 1
#define SEARCH 2
#define APPROACH_NET 3
#define SHOOT 4
#define PAUSE 5
#define LOCALIZATION_A 6
#define APPROACH_HOME 7
#define LOCALIZATION_D 8
#define DEFENSE 9


#define angel_tol 160
#define threashold 600
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
int actFlag=0;
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

void shoot(){
	 m3pi.forward(1);
    wait_ms(100);
    m3pi.stop();
    wait_ms(20);
}
void stop(){
	 m3pi.stop();
	 wait(5);
}

void rotate(int deg){
	//int time=0;
	
	//time=(int)(25/4*(abs(deg)));
	if(deg<0){
		m3pi.left(0.3);
		wait_ms(30);
	}
	else{
		m3pi.right(0.3);
		wait_ms(30);
	}
	m3pi.stop();
	wait_ms(1);
}
void forward(int dist){
	int time=(int)(14*dist);
	m3pi.forward(0.6);
	wait_ms(time);
	m3pi.stop();
	wait_ms(1);
}

void go_forward(){
	m3pi.forward(0.5);
	wait_ms(300);
	m3pi.stop();
	wait(1);
	m3pi.backward(0.5);
	wait_ms(300);
	m3pi.stop();
	wait_ms(10);
}

void go_left(){
		m3pi.left(0.3);
	wait_ms(150);
		m3pi.stop();
	wait_ms(10);
	m3pi.forward(0.5);
	wait_ms(300);
	m3pi.stop();
	wait(1);
	m3pi.backward(0.5);
	wait_ms(300);
		m3pi.stop();
	wait_ms(10);
	m3pi.right(0.3);
	wait_ms(150);
	m3pi.stop();
	wait_ms(10);
	
}

void go_right(){
			m3pi.right(0.3);
	wait_ms(150);
		m3pi.stop();
	wait_ms(10);
	m3pi.forward(0.5);
	wait_ms(300);
	m3pi.stop();
	wait(1);
	m3pi.backward(0.5);
	wait_ms(300);
		m3pi.stop();
	wait_ms(10);
	m3pi.left(0.3);
	wait_ms(150);
	m3pi.stop();
	wait_ms(10);
	
	
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
		//      clock_init();
		//	printf("clock ready\n");
		 //     cam_init();	
		//	printf("cam ready\n");
	  //pc.baud(9600);
	
			bmac_task_config();
			nrk_create_taskset();
		  state=DEFENSE;
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
	
  // Enable AES 128 bit encryption
  // When encryption is active, messages from plaintext
  // source will still be received. 
	
	// Commented out by MB
  // bmac_encryption_set_key(aes_key,16);
  // bmac_encryption_enable();
	// bmac_encryption_disable();

	
  // By default the RX check rate is 200ms
  // below shows how to change that
  //check_period.secs=0;
  //check_period.nano_secs=200*NANOS_PER_MS;
  //val=bmac_set_rx_check_rate(check_period);

  // The default Clear Channel Assement RSSI threshold.
  // Setting this value higher means that you will only trigger
  // receive with a very strong signal.  Setting this lower means
  // bmac will try to receive fainter packets.  If the value is set
  // too high or too low performance will suffer greatly.
   bmac_set_cca_thresh(DEFAULT_BMAC_CCA); 


  // This sets the next RX buffer.
  // This can be called at anytime before releaseing the packet
  // if you wish to do a zero-copy buffer switch
  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

  while (1) {
    // Wait until an RX packet is received
    val = bmac_wait_until_rx_pkt ();
		
    // Get the RX packet 
    nrk_led_set (ORANGE_LED);
    local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
    //if( bmac_rx_pkt_is_encrypted()==1 ) nrk_kprintf( PSTR( "Packet Encrypted\r\n" ));
 //   printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
    //for (i = 0; i < len; i++)
    //  printf ("%c", rx_buf[i]);
 //   printf ("]\r\n");
		if(rx_buf[2]=='2'&&actFlag!=2) {
			state=PAUSE;
			actFlag=2;
		}
    nrk_led_clr (ORANGE_LED);
   
		// Release the RX buffer so future packets can arrive 
    bmac_rx_pkt_release ();
		
		// this is necessary
    nrk_wait_until_next_period ();

  }

}


uint8_t ctr_cnt[4];

void tx_task ()
{
  uint8_t j, i, val, len, cnt;
  int8_t v;
  nrk_sig_t tx_done_signal;
  nrk_sig_mask_t ret;
  nrk_time_t r_period;
	
	// printf("tx_task PID=%d\r\n", nrk_get_pid ());

  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
  while (!bmac_started ())
    nrk_wait_until_next_period ();
  

  // Sample of using Reservations on TX packets
  // This example allows 2 packets to be sent every 5 seconds
  // r_period.secs=5;
  // r_period.nano_secs=0;
  // v=bmac_tx_reserve_set( &r_period, 2 );
  // if(v==NRK_ERROR) nrk_kprintf( PSTR("Error setting b-mac tx reservation (is NRK_MAX_RESERVES defined?)\r\n" ));


  // Get and register the tx_done_signal if you want to
  // do non-blocking transmits
  tx_done_signal = bmac_get_tx_done_signal ();
  nrk_signal_register (tx_done_signal);

  ctr_cnt[0]=0; ctr_cnt[1]=0; ctr_cnt[2]=0; ctr_cnt[3]=0;
  cnt = 0;
	
  while (1) {
    // Build a TX packet
//			set_adc_chan(0);
//		uint16_t ad0=get_adc_val();
	//	set_adc_chan(1);
//		uint16_t ad1=get_adc_val();
//		uint16_t ad0=1;
//		uint16_t ad1=2;
//		printf("AD0: %u, AD1: %u\n",ad0,ad1);
	
switch (state){

case DEFENSE:	
	actFlag=0;
				set_adc_chan(0);
		uint16_t ad0=get_adc_val();
		set_adc_chan(1);
		uint16_t ad1=get_adc_val();
		
		
		printf("AD0: %u, AD1: %u\n",ad0,ad1);
		if(ad0<threashold||ad1<threashold){
			if(abs(ad0-ad1)<60&&(ad0<angel_tol||ad1<angel_tol)){
				go_forward();
			
			}
			else if (ad0>ad1){
				m3pi.left(0.15);
				wait_ms(1);
	//			printf("left\n");

			}
			else if(ad0<ad1){
			m3pi.right(0.15);
				wait_ms(1);
	//			printf("right\n");
			}		
		}
		else{
			m3pi.stop();
			wait_ms(5);
		}
		
break;

case PAUSE:
	  
		 m3pi.stop();
		 wait(4);
	//	 m3pi.right(0.5);
	//	 wait_ms(200);
//		 m3pi.stop();
	//	 wait(2);
     state=DEFENSE;
     actFlag=99;
	   break;
    
}
	
    nrk_led_set (BLUE_LED);
		
    // Auto ACK is an energy efficient link layer ACK on packets
    // If Auto ACK is enabled, then bmac_tx_pkt() will return failure
    // if no ACK was received. In a broadcast domain, the ACK's will
    // typically collide.  To avoid this, one can use address decoding. 
    // The functions are as follows:
    // bmac_auto_ack_enable();
			 bmac_auto_ack_disable();

    // Address decoding is a way of preventing the radio from receiving
    // packets that are not address to a particular node.  This will 
    // supress ACK packets from nodes that should not automatically ACK.
    // The functions are as follows:
    // bmac_addr_decode_set_my_mac(uint16_t MAC_ADDR); 
    // bmac_addr_decode_dest_mac(uint16_t DST_ADDR);  // 0xFFFF is broadcast
    // bmac_addr_decode_enable();
    // bmac_addr_decode_disable();
/*
     ctr_cnt[0]=cnt; 
     if(ctr_cnt[0]==255) ctr_cnt[1]++; 
     if(ctr_cnt[1]==255) ctr_cnt[2]++; 
     if(ctr_cnt[2]==255) ctr_cnt[3]++; 
     // You need to increase the ctr on each packet to make the 
     // stream cipher not repeat.
     bmac_encryption_set_ctr_counter(&ctr_cnt,4);

*/  // For blocking transmits, use the following function call.
    // For this there is no need to register  
     val=bmac_tx_pkt(tx_buf, strlen(tx_buf));
		 if(val==NRK_OK) cnt++;
 //		 else printf("NO ack or Reserve Violated! \r\n");

    // This function shows how to transmit packets in a
    // non-blocking manner  
    // val = bmac_tx_pkt_nonblocking(tx_buf, strlen (tx_buf));
    // printf ("Tx packet enqueued\r\n");
    // This functions waits on the tx_done_signal
    //ret = nrk_event_wait (SIG(tx_done_signal));

    // Just check to be sure signal is okay
    //if(ret & SIG(tx_done_signal) == 0 ) 
    //printf ("TX done signal error\r\n");
   
    // If you want to see your remaining reservation
    // printf( "reserve=%d ",bmac_tx_reserve_get() );
    
    // Task gets control again after TX complete
//    printf("Tx task sent data!\r\n");
    nrk_led_clr (BLUE_LED);
//		printf("tx_task PID=%d\r\n", nrk_get_pid ());
    nrk_wait_until_next_period ();
  }

}

void nrk_create_taskset ()
{


  RX_TASK.task = rx_task;
  nrk_task_set_stk( &RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 2;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
  RX_TASK.period.secs = 6;
  RX_TASK.period.nano_secs = 0;
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

//  printf ("Create done\r\n");
}