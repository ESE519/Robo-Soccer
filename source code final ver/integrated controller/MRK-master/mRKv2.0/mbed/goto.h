#include <math.h>
//#include "m3pi.h"
//#include "mbed.h"
//m3pi m3pi(p23,p9,p10);

int approach_net(int Xnet, int Ynet){

		double Xnet_bot; double Ynet_bot;

		Xnet_bot = Xnet - XBOT;		// vector from bot to net
		Ynet_bot = Ynet - YBOT;		// ^

		net_dist = Xnet_bot*Xnet_bot + Ynet_bot*Ynet_bot;		// distance to net
		net_dist = sqrt(net_dist);								// ^
    beta=atan(abs((double)Xnet_bot)/abs((double)Ynet_bot));
	  beta=abs(beta);
	 if(Xnet_bot>0&&Ynet_bot>0)
			return 1;
		else if(Xnet_bot>0&&Ynet_bot<0)
			return 2;
	  else if(Xnet_bot<0&&Ynet_bot>0)
			return 3;
		else if(Xnet_bot<0&&Ynet_bot<0)
			return 4;
	

	}		
