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
#define angel_tol 150
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
int tx_count=0;
int score_a = 0;
int score_b = 0;
int ir01_count = 0;
int ir23_count = 0;
int jamCount=0;
int current_cnt=1;
// 7 segment display code
int numb[10][7] = {{0, 0, 0, 0, 0, 0, 1},  //0
                {1, 0, 0, 1, 1, 1, 1},     //1
                {0, 0, 1, 0, 0, 1, 0},     //2
                {0, 0, 0, 0, 1, 1, 0},     //3
                {1, 0, 0, 1, 1, 0, 0},     //4
                {0, 1, 0, 0, 1, 0, 0}, 
                {0, 1, 0, 0, 0, 0, 0}, 
                {0, 0, 0, 1, 1, 1, 1}, 
                {0, 0, 0, 0, 0, 0, 0}, 
                {0, 0, 0, 1, 1, 0, 0}};    //9

//#include "position.h"
//#include "wiicamera.h"
//#include "goto.h"
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
#include "beep.h"

DigitalOut led_switch[] = {p6, p7}; 
DigitalOut ledPins[] = {p30, p29, p28, p27, p8, p25, p24};
Beep buzzer(p26);

// Only require MAC address for address decode 
//#define MAC_ADDR	0x0001

//m3pi m3pi(p23,p9,p10);
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

void play(char note) {
    if (note=='a') {
        buzzer.beep(880,0.1);
    }
    if (note=='b') {
        buzzer.beep(987,0.1);
    }
    if (note=='c') {
        buzzer.beep(1024,0.1);
    }
    if (note=='d') {
        buzzer.beep(1175,0.1);
    }
    if (note=='e') {
        buzzer.beep(1319,0.1);
    }
    if (note=='f') {
        buzzer.beep(1397,0.1);
    }
    if (note=='g') {
        buzzer.beep(1568,0.1);
    }
		if (note=='h') {
        buzzer.beep(1739,0.1);
    }
		if (note=='i') {
        buzzer.beep(1910,0.1);
    }
		if (note=='j') {
        buzzer.beep(2081,0.3);
    }

    if (note=='w') {
        wait(0.05);
    }
    wait (0.1); //wait while the note plays.
}

void popcorn() {

     play('a');
	   play('b');
	   play('c');
	   play('d');
	   play('e');
	   play('f');
	   play('g');
	   play('h');
	   play('i');
	   play('j');
	   
}

int main(void)

  {
			nrk_setup_ports();
	
			nrk_init();
	
		  init_adc();
	
			bmac_task_config();
			nrk_create_taskset();
		  popcorn();
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
		// test 7 segment
		for(int i = 0; i < 7; i++)
    {
      //clear all pins
      ledPins[i] = 1;
    } 

    // display score a
		led_switch[0] = 1;
		led_switch[1] = 0;
    for(int i = 0; i < 7; i++)
    {
      ledPins[i] = numb[score_a][i];
    }
		wait_ms(500);
		
		// display score b
		led_switch[0] = 0;
		led_switch[1] = 1;
    for(int i = 0; i < 7; i++)
    {
      ledPins[i] = numb[score_b][i];
    }	
    wait_ms(500);
		
    // Wait until an RX packet is received
    val = bmac_wait_until_rx_pkt ();
    // Get the RX packet 
    nrk_led_set (ORANGE_LED);
    local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
    //if( bmac_rx_pkt_is_encrypted()==1 ) nrk_kprintf( PSTR( "Packet Encrypted\r\n" ));
 //   printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
  //  for (i = 0; i < len; i++)
    //  printf ("%c", rx_buf[i]);
 //   printf ("]\r\n");
    nrk_led_clr (ORANGE_LED);
   
		// Release the RX buffer so future packets can arrive 
    bmac_rx_pkt_release ();
		
		// this is necessary
    nrk_wait_until_next_period ();

  }

}


uint8_t ctr_cnt[4];
void transmit(int flag){
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
	if(flag==0){
	   sprintf(tx_buf,"G0%d\n",2);
	}
	else if(flag==1){
		sprintf(tx_buf,"G0%d%d\n",6,current_cnt);
	}
	else if(flag==2){
		sprintf(tx_buf,"G0%d%d%d\n",7,score_a,score_b);
	}
	   bmac_auto_ack_disable();  
     val=bmac_tx_pkt(tx_buf, strlen(tx_buf));
		 if(val==NRK_OK) cnt++;
}


void tx_task ()
{
  nrk_led_set (BLUE_LED);
	
	// printf("tx_task PID=%d\r\n", nrk_get_pid ());

  // Wait until the tx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
//  while (!bmac_started ())
//    nrk_wait_until_next_period ();
  

  // Sample of using Reservations on TX packets
  // This example allows 2 packets to be sent every 5 seconds
  // r_period.secs=5;
  // r_period.nano_secs=0;
  // v=bmac_tx_reserve_set( &r_period, 2 );
  // if(v==NRK_ERROR) nrk_kprintf( PSTR("Error setting b-mac tx reservation (is NRK_MAX_RESERVES defined?)\r\n" ));


  // Get and register the tx_done_signal if you want to
  // do non-blocking transmits
//  tx_done_signal = bmac_get_tx_done_signal ();
//  nrk_signal_register (tx_done_signal);

 // ctr_cnt[0]=0; ctr_cnt[1]=0; ctr_cnt[2]=0; ctr_cnt[3]=0;
//  cnt = 0;
	
  while (1) {
		//chanell 0-3 are for goal sensors, chanell 4 for dejam button, chanell 5 for game start button
	  set_adc_chan(0);
		uint16_t ad0=get_adc_val();
		set_adc_chan(1);
		uint16_t ad1=get_adc_val();
	  set_adc_chan(2);
		uint16_t ad2=get_adc_val();
		set_adc_chan(3);
		uint16_t ad3=get_adc_val();
		set_adc_chan(4);
		uint16_t ad4=get_adc_val();
		set_adc_chan(5);
		uint16_t ad5=get_adc_val();
		printf("AD0: %u, AD1: %u, AD2: %u, AD3: %u,AD4: %u, AD5: %u\n",ad0,ad1,ad2,ad3,ad4,ad5);
 
	if(ad0<angel_tol||ad1<angel_tol){ 
		ir01_count++;
    if(ir01_count == 3) {
		ir01_count=0;
	  if(tx_count<1) {
			score_a++;
		  popcorn();
		}
		if(tx_count<5){
			transmit(0);//0 for goal
		 //sprintf(tx_buf,"G0%d\n",2);  //3 stands for goal
	  //		printf("goal!!\n");
		 tx_count++;
	 }
  }
 }
 else if(ad2<angel_tol||ad3<angel_tol){ 
	  ir23_count++;
	  if(ir23_count == 3) {
	  ir23_count=0;
	  if(tx_count<1) { 
  	 score_b++;
		 popcorn();
		}
		if(tx_count<5){
			transmit(0);//0 for goal
	//	 sprintf(tx_buf,"G0%d\n",2);  //3 stands for goal
	//		printf("goal!!\n");
		 tx_count++;
	 }
  }
 }
 //else if(ad0>300&&ad1>300&&ad2>300&&ad3>300) {
	else{
		tx_count=0;
	//  sprintf(tx_buf,"G0%d%d%d\n",5,score_a,score_b);
		ir01_count=0;
		ir23_count=0;
 }
   

	if(ad4<100){
		transmit(1);//1 for dejam		
	}
	if(ad4>120)
			current_cnt++;
	
	if(ad5<100)
		transmit(2);//2 for start
		 wait_ms(10);
	printf("current cnt:%d\n",current_cnt);
    // Auto ACK is an energy efficient link layer ACK on packets
    // If Auto ACK is enabled, then bmac_tx_pkt() will return failure
    // if no ACK was received. In a broadcast domain, the ACK's will
    // typically collide.  To avoid this, one can use address decoding. 
    // The functions are as follows:
    // bmac_auto_ack_enable();
	//		 bmac_auto_ack_disable();

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
 //if(val==NRK_OK) cnt++;
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
  TX_TASK.period.nano_secs = 200*NANOS_PER_MS;
  TX_TASK.cpu_reserve.secs = 1;
  TX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);

//  printf ("Create done\r\n");
}