#include "mr32.h"
#include <math.h>

void rotateRel_basic(int speed, double deltaAngle);
void getBacon();
void goTo();
void turnWall(int state);
void turnCorner(int state);
int checkCollision();
void moveOutTheWay(int dist);

char cornerFlag=0;
int baconRotate=0.1;
int groundSensor;
double x, y, t;

int main(void){
	initPIC32(); //initializa o software do microrato
	closedLoopControl(true);
	setVel2(0, 0);
	while(1){
		while(!startButton());
		wait (1);
      	enableObstSens();
    	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
	    setServoPos(0);
	    leds(0);

	    do{
	    	getBacon();
	    	while(!groundSensor) {
	    		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    		readAnalogSensors();                // Fill in "analogSensors" structure
	    		goTo(0);
	    		

         		groundSensor = readLineSensors(0);	// Read ground sensor	
	    	}
	    	leds(15);
			setVel2(0, 0);
			wait(5);
			waitTick40ms();                    // Wait for next 40ms tick (sensor provides a new value each 40 ms)
         	readAnalogSensors();				// Fill in "analogSensors" structure
			printf("Dist_left=%03d, Dist_center=%03d, Dist_right=%03d, Bat_voltage=%03d\n", 
               analogSensors.obstSensLeft,
               analogSensors.obstSensFront, 
               analogSensors.obstSensRight, 
               analogSensors.batteryVoltage);

	    	break;
	    }while(!stopButton());
	}

}

int checkCollision() {
	waitTick40ms();                    // Wait for next 40ms tick (sensor provides a new value each 40 ms)
 	readAnalogSensors();				// Fill in "analogSensors" structure
 	if(analogSensors.obstSensFront<10) return 1;
 	else return 0;

}


void getBacon() {
	while(!readBeaconSens()) setVel2(-20,20);
	setVel2(0, 0);
}

void moveOutTheWay(int dist) {
	getRobotPos(&x, &y, &t);
	double xtmp, ytmp, tmp, pos, xinit, yinit;
	xinit=x;
	yinit=y;
    setVel2(40,40);
    while(pos<dist-3){
    	getRobotPos(&x, &y, &t);
    	xtmp=fabs(xinit-x);
    	ytmp=fabs(yinit-y);
		tmp=pow(xtmp,2);
	    pos=pow(ytmp,2);
	    pos+=tmp;
	    pos=sqrt(pos);
	    pos=fabs(pos);
    }
    setVel2(0,0);

}

void turnCorner(int state) {
	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	readAnalogSensors();                // Fill in "analogSensors" structure
	if(checkCollision()) return;
	moveOutTheWay(300);
	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	readAnalogSensors();                // Fill in "analogSensors" structure
	if(checkCollision()) return;
	if(state==-1) {
		setVel2(0,50);
		wait(7);
		setVel2(0,0);
		if(analogSensors.obstSensFront>30) {
			moveOutTheWay(250);
		}
	}
	else {
		rotateRel_basic(-30, M_PI/2);
		if(analogSensors.obstSensFront>30) {
			moveOutTheWay(250);
		}
	}
}

void turnWall(int state) {
	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	readAnalogSensors();                // Fill in "analogSensors" structure
	while(analogSensors.obstSensFront<30){
		
		setVel2(30*state, -30*state);
		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
	}

	waitTick40ms();                    // Wait for next 40ms tick (sensor provides a new value each 40 ms)
 	readAnalogSensors();				// Fill in "analogSensors" structure

	if(state==-1) {
		while(1) {
			if(checkCollision()) break;
			if(analogSensors.obstSensRight>30 && analogSensors.obstSensRight<40) {
				cornerFlag=0;
				setVel2(40,10);
			}
			else if(analogSensors.obstSensRight<20) {
				cornerFlag=0;
				setVel2(10,40);
			}
			else if(analogSensors.obstSensRight>40) {
				if(cornerFlag){
					turnCorner(1);
					getBacon();
					break;
				}
				else {
					cornerFlag=1;
					delay(500);
				}
			}
			else if(analogSensors.obstSensFront<25){
				cornerFlag=0;
				rotateRel_basic(-30,M_PI/2);
			}
			else {
				cornerFlag=0;
				setVel2(30, 30);
			}
		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
		}
	}
	if(state==1) {
			while(1) {
			if(checkCollision()) break;
			if(analogSensors.obstSensLeft>30 && analogSensors.obstSensLeft<40) {
				cornerFlag=0;
				setVel2(10,40);
			}
			else if(analogSensors.obstSensLeft<20) {
				cornerFlag=0;
				setVel2(40,10);
			}
			else if(analogSensors.obstSensLeft>40) {
				if(cornerFlag){
					turnCorner(-1);
					getBacon();
					break;
				}
				else cornerFlag=1;
			}
			else if(analogSensors.obstSensFront<25){
				cornerFlag=0;
				rotateRel_basic(30,M_PI/2);
			}
			else {
				cornerFlag=0;
				setVel2(30, 30);
			}
		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
		}
	}
}

void goTo(int state) {
	double tmp, dr, dl;
	if(state==0) {
        waitTick40ms();                    // Wait for next 40ms tick (sensor provides a new value each 40 ms)
        readAnalogSensors();				// Fill in "analogSensors" structure
        dr=pow(analogSensors.obstSensFront,2);
        dl=dr;
        tmp=pow(analogSensors.obstSensRight,2);
        dr+=tmp;
        tmp=pow(analogSensors.obstSensLeft,2);
        dl+=tmp;
        dr=sqrt(dr);
        dl=sqrt(dl);
        if(dr>=25 && dl>=25) {
			setVel2(30, 30);
        }

        else {
         	if(analogSensors.obstSensLeft<25) {
         		turnWall(1);
         	}
         	else if(analogSensors.obstSensRight<25) {
         		turnWall(-1);
         	}
         	else rotateRel_basic(30, M_PI);
         }
	}
}

void rotateRel_basic(int speed, double deltaAngle)
{
   double x, y, t;
   double targetAngle;
   double error;
   int cmdVel, errorSignOri;

   getRobotPos(&x, &y, &t);
   targetAngle = normalizeAngle(t + deltaAngle);
   error = normalizeAngle(targetAngle - t);
   errorSignOri = error < 0 ? -1 : 1;

   cmdVel = error < 0 ? -speed : speed;
   setVel2(-cmdVel, cmdVel);

   do
   {
      getRobotPos(&x, &y, &t);
      error = normalizeAngle(targetAngle - t);
   } while (fabs(error) > 0.01 && errorSignOri * error > 0);
   setVel2(0, 0);
}

