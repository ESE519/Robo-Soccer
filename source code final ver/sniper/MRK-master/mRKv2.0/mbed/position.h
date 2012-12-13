#include <math.h>
#include <stdio.h>

#define V_vert 35   // distance between the vertically-arranged stars [cm]
#define x_center 512 // x-direction center offset of the Wii sensor [pixels]
#define y_center 384 // y-direction center offset of the Wii sensor [pixels]




void track(unsigned int *blob){
// use the mWii data to find the position and orientation of the bot relative to rink center

// SUBROUTINE VARIABLES
//int x1; int x2; int x3; int x4; int y1; int y2; int y3; int y4;
//long v21; long v31; long v41; long v32; long v42; long v43;

	int x1; int x2; int x3; int x4; int y1; int y2; int y3; int y4;
long v21; long v31; long v41; long v32; long v42; long v43;
double v_vert;
int x_vert1; int x_vert2; int y_vert1; int y_vert2;
double scale;
double xO; double yO;
int x_other1; int x_other2;
int x_actually2; int x_actually3; int y_actually2; int y_actually3;
double alpha; double phi; double r;

x1 = blob[0] - x_center;	// pull variables from IR blobs, shift using center offset
y1 = blob[1] - y_center;	// ^
x2 = blob[3] - x_center;	// ^
y2 = blob[4] - y_center;	// ^
x3 = blob[6] - x_center;	// ^
y3 = blob[7] - y_center;	// ^
x4 = blob[9] - x_center;	// ^
y4 = blob[10] - y_center;	// ^

v21 = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);	// these are actually the squares of the vector magnitudes
v31 = (x3-x1)*(x3-x1) + (y3-y1)*(y3-y1);	// ^
v41 = (x4-x1)*(x4-x1) + (y4-y1)*(y4-y1);	// ^
v32 = (x3-x2)*(x3-x2) + (y3-y2)*(y3-y2);	// ^
v42 = (x4-x2)*(x4-x2) + (y4-y2)*(y4-y2);	// ^
v43 = (x4-x3)*(x4-x3) + (y4-y3)*(y4-y3);	// ^

// determine the longest inter-blob vector (this corresponds to the IR blob vertical)
if( v21>v31 && v21>v41 && v21>v32 && v21>v42 && v21>v43 ){ v_vert = v21; x_vert1 = x1; x_vert2 = x2; y_vert1 = y1; y_vert2 = y2; }
else if( v31>v21 && v31>v41 && v31>v32 && v31>v42 && v31>v43 ){ v_vert = v31; x_vert1 = x1; x_vert2 = x3; y_vert1 = y1; y_vert2 = y3; }
else if( v41>v21 && v41>v31 && v41>v32 && v41>v42 && v41>v43 ){ v_vert = v41; x_vert1 = x1; x_vert2 = x4; y_vert1 = y1; y_vert2 = y4; }
else if( v32>v21 && v32>v31 && v32>v41 && v32>v42 && v32>v43 ){ v_vert = v32; x_vert1 = x2; x_vert2 = x3; y_vert1 = y2; y_vert2 = y3; }
else if( v42>v21 && v42>v31 && v42>v41 && v42>v32 && v42>v43 ){ v_vert = v42; x_vert1 = x2; x_vert2 = x4; y_vert1 = y2; y_vert2 = y4; }
else if( v43>v21 && v43>v31 && v43>v41 && v43>v32 && v43>v42 ){ v_vert = v43; x_vert1 = x3; x_vert2 = x4; y_vert1 = y3; y_vert2 = y4; }
else{ v_vert = 1; x_vert1 = 0; x_vert2 = 0; y_vert1 = 0; y_vert2 = 0; }
v_vert = sqrt(v_vert);

scale = V_vert/v_vert;	// convert all variables from [pixels] to [cm]
x1 = scale*x1;	// ^
x2 = scale*x2;	// ^
x3 = scale*x3;	// ^
x4 = scale*x4;	// ^
y1 = scale*y1;	// ^
y2 = scale*y2;	// ^
y3 = scale*y3;	// ^
y4 = scale*y4;	// ^
x_vert1 = scale*x_vert1;	// ^
x_vert2 = scale*x_vert2;	// ^
y_vert1 = scale*y_vert1;	// ^
y_vert2 = scale*y_vert2;	// ^

xO = (x_vert1 + x_vert2) / 2.0;	// define the local position of the origin
yO = (y_vert1 + y_vert2) / 2.0;	// ^

// determine the shortest inter-blob vector (this corresponds to V31)
if( v21<v31 && v21<v41 && v21<v32 && v21<v42 && v21<v43 ){ x_other1 = x1; x_other2 = x2; }
else if( v31<v21 && v31<v41 && v31<v32 && v31<v42 && v31<v43 ){ x_other1 = x1; x_other2 = x3; }
else if( v41<v21 && v41<v31 && v41<v32 && v41<v42 && v41<v43 ){ x_other1 = x1; x_other2 = x4; }
else if( v32<v21 && v32<v31 && v32<v41 && v32<v42 && v32<v43 ){ x_other1 = x2; x_other2 = x3; }
else if( v42<v21 && v42<v31 && v42<v41 && v42<v32 && v42<v43 ){ x_other1 = x2; x_other2 = x4; }
else if( v43<v21 && v43<v31 && v43<v41 && v43<v32 && v43<v42 ){ x_other1 = x3; x_other2 = x4; }
else{ x_other1 = 0; x_other2 = 0; }

// determine the blobs 1, 2, and 3
if( x_vert1 == x_other1 ){ x_actually2 = x_vert2; x_actually3 = x_vert1; y_actually2 = y_vert2; y_actually3 = y_vert1; }
else if( x_vert2 == x_other1 ){ x_actually2 = x_vert1; x_actually3 = x_vert2; y_actually2 = y_vert1; y_actually3 = y_vert2; }
else if( x_vert1 == x_other2 ){ x_actually2 = x_vert2; x_actually3 = x_vert1; y_actually2 = y_vert2; y_actually3 = y_vert1; }
else if( x_vert2 == x_other2 ){ x_actually2 = x_vert1; x_actually3 = x_vert2; y_actually2 = y_vert1; y_actually3 = y_vert2; }
else{ x_actually2 = 0; x_actually3 = 0; y_actually2 = 0; y_actually3 = 0; }

// find the angle between the local frame and the global frame
theta = atan2( (double)(x_actually3 - x_actually2), (double)(y_actually3 - y_actually2) );

// find the magnitude of the vector to the origin
r = xO*xO + yO*yO;
r = sqrt(r);

// find the angle between the local x-axis and the vector to the origin
alpha = -atan2(yO,xO);

// find the angle between the global x-axis and the vector to the origin
phi = theta - alpha;

XBOT = -r*cos(phi);	// determine the bot's global position
YBOT = -r*sin(phi);	// ^
}	


void track_deg(unsigned int *blob){
		int x1; int x2; int x3; int x4; int y1; int y2; int y3; int y4;
long v21; long v31; long v41; long v32; long v42; long v43;
double v_vert;
int x_vert1; int x_vert2; int y_vert1; int y_vert2;
double scale;
double xO; double yO;
int x_other1; int x_other2;
int x_actually2; int x_actually3; int y_actually2; int y_actually3;
double alpha; double phi; double r;

x1 = blob[0] - x_center;	// pull variables from IR blobs, shift using center offset
y1 = blob[1] - y_center;	// ^
x2 = blob[3] - x_center;	// ^
y2 = blob[4] - y_center;	// ^
x3 = blob[6] - x_center;	// ^
y3 = blob[7] - y_center;	// ^
x4 = blob[9] - x_center;	// ^
y4 = blob[10] - y_center;	// ^

v21 = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);	// these are actually the squares of the vector magnitudes
v31 = (x3-x1)*(x3-x1) + (y3-y1)*(y3-y1);	// ^
v41 = (x4-x1)*(x4-x1) + (y4-y1)*(y4-y1);	// ^
v32 = (x3-x2)*(x3-x2) + (y3-y2)*(y3-y2);	// ^
v42 = (x4-x2)*(x4-x2) + (y4-y2)*(y4-y2);	// ^
v43 = (x4-x3)*(x4-x3) + (y4-y3)*(y4-y3);	// ^

// determine the longest inter-blob vector (this corresponds to the IR blob vertical)
if( v21>v31 && v21>v41 && v21>v32 && v21>v42 && v21>v43 ){ v_vert = v21; x_vert1 = x1; x_vert2 = x2; y_vert1 = y1; y_vert2 = y2; }
else if( v31>v21 && v31>v41 && v31>v32 && v31>v42 && v31>v43 ){ v_vert = v31; x_vert1 = x1; x_vert2 = x3; y_vert1 = y1; y_vert2 = y3; }
else if( v41>v21 && v41>v31 && v41>v32 && v41>v42 && v41>v43 ){ v_vert = v41; x_vert1 = x1; x_vert2 = x4; y_vert1 = y1; y_vert2 = y4; }
else if( v32>v21 && v32>v31 && v32>v41 && v32>v42 && v32>v43 ){ v_vert = v32; x_vert1 = x2; x_vert2 = x3; y_vert1 = y2; y_vert2 = y3; }
else if( v42>v21 && v42>v31 && v42>v41 && v42>v32 && v42>v43 ){ v_vert = v42; x_vert1 = x2; x_vert2 = x4; y_vert1 = y2; y_vert2 = y4; }
else if( v43>v21 && v43>v31 && v43>v41 && v43>v32 && v43>v42 ){ v_vert = v43; x_vert1 = x3; x_vert2 = x4; y_vert1 = y3; y_vert2 = y4; }
else{ v_vert = 1; x_vert1 = 0; x_vert2 = 0; y_vert1 = 0; y_vert2 = 0; }
v_vert = sqrt(v_vert);

scale = V_vert/v_vert;	// convert all variables from [pixels] to [cm]
x1 = scale*x1;	// ^
x2 = scale*x2;	// ^
x3 = scale*x3;	// ^
x4 = scale*x4;	// ^
y1 = scale*y1;	// ^
y2 = scale*y2;	// ^
y3 = scale*y3;	// ^
y4 = scale*y4;	// ^
x_vert1 = scale*x_vert1;	// ^
x_vert2 = scale*x_vert2;	// ^
y_vert1 = scale*y_vert1;	// ^
y_vert2 = scale*y_vert2;	// ^

xO = (x_vert1 + x_vert2) / 2.0;	// define the local position of the origin
yO = (y_vert1 + y_vert2) / 2.0;	// ^

// determine the shortest inter-blob vector (this corresponds to V31)
if( v21<v31 && v21<v41 && v21<v32 && v21<v42 && v21<v43 ){ x_other1 = x1; x_other2 = x2; }
else if( v31<v21 && v31<v41 && v31<v32 && v31<v42 && v31<v43 ){ x_other1 = x1; x_other2 = x3; }
else if( v41<v21 && v41<v31 && v41<v32 && v41<v42 && v41<v43 ){ x_other1 = x1; x_other2 = x4; }
else if( v32<v21 && v32<v31 && v32<v41 && v32<v42 && v32<v43 ){ x_other1 = x2; x_other2 = x3; }
else if( v42<v21 && v42<v31 && v42<v41 && v42<v32 && v42<v43 ){ x_other1 = x2; x_other2 = x4; }
else if( v43<v21 && v43<v31 && v43<v41 && v43<v32 && v43<v42 ){ x_other1 = x3; x_other2 = x4; }
else{ x_other1 = 0; x_other2 = 0; }

// determine the blobs 1, 2, and 3
if( x_vert1 == x_other1 ){ x_actually2 = x_vert2; x_actually3 = x_vert1; y_actually2 = y_vert2; y_actually3 = y_vert1; }
else if( x_vert2 == x_other1 ){ x_actually2 = x_vert1; x_actually3 = x_vert2; y_actually2 = y_vert1; y_actually3 = y_vert2; }
else if( x_vert1 == x_other2 ){ x_actually2 = x_vert2; x_actually3 = x_vert1; y_actually2 = y_vert2; y_actually3 = y_vert1; }
else if( x_vert2 == x_other2 ){ x_actually2 = x_vert1; x_actually3 = x_vert2; y_actually2 = y_vert1; y_actually3 = y_vert2; }
else{ x_actually2 = 0; x_actually3 = 0; y_actually2 = 0; y_actually3 = 0; }

// find the angle between the local frame and the global frame
theta = atan2( (double)(x_actually3 - x_actually2), (double)(y_actually3 - y_actually2) );

// find the magnitude of the vector to the origin
//r = xO*xO + yO*yO;
//r = sqrt(r);

// find the angle between the local x-axis and the vector to the origin
//alpha = -atan2(yO,xO);

// find the angle between the global x-axis and the vector to the origin
//phi = theta - alpha;

//XBOT = -r*cos(phi);	// determine the bot's global position
//YBOT = -r*sin(phi);	// ^
}	
