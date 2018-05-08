#include "mr32.h"
#include <math.h>

void rotateRel_basic(int speed, double deltaAngle);
void getBacon();
void goTo();
void turnWall(int state);
void turnCorner(int state);
int checkCollision();

int baconRotate=0.1;
int groundSensor;
int speedL, speedR;

int main(void){
    double x, y, t;
	initPIC32(); //initializa o software do microrato
	closedLoopControl(true);
	speedL=0;
	speedR=0;
	setVel2(speedL, speedR);
	while(1){
		while(!startButton());
		wait (1);
		speedL=0;
		speedR=0;
		setVel2(speedL, speedR);

      	enableObstSens();
    	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
	    setServoPos(0);
	    leds(0);

	    do{
	    	getBacon();
	    	while(1) {
	    		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    		readAnalogSensors();                // Fill in "analogSensors" structure
	    		goTo(0);

         		groundSensor = readLineSensors(0);	// Read ground sensor	
	    	}
	    	leds(15);
	    	speedL=0;
			speedR=0;
			setVel2(speedL, speedR);
			wait(5);
			waitTick40ms();                    // Wait for next 40ms tick (sensor provides a new value each 40 ms)
         	readAnalogSensors();				// Fill in "analogSensors" structure
			printf("Dist_left=%03d, Dist_center=%03d, Dist_right=%03d, Bat_voltage=%03d\n", 
               analogSensors.obstSensLeft,
               analogSensors.obstSensFront, 
               analogSensors.obstSensRight, 
               analogSensors.batteryVoltage);
	    }while(!stopButton());
	}

}

int checkCollision() {
	waitTick40ms();                    // Wait for next 40ms tick (sensor provides a new value each 40 ms)
 	readAnalogSensors();				// Fill in "analogSensors" structure
 	if(analogSensors.obstSensFront<20) return 1;
 	else return 0;

}

void turnCorner(int state) {
	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	readAnalogSensors();                // Fill in "analogSensors" structure
	if(checkCollision()) return;
	setVel2(40,40);
	wait(10);
	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	readAnalogSensors();                // Fill in "analogSensors" structure
	if(checkCollision()) return;
	if(state==-1) {
		rotateRel_basic(30, M_PI/2);
		if(analogSensors.obstSensFront>30) {
			setVel2(40,40);
			wait(15);
		}
	}
	else {
		rotateRel_basic(30, -M_PI/2);
		if(analogSensors.obstSensFront>30) {
			setVel2(40,40);
			wait(15);
		}
	}
}

void getBacon() {
	while(!readBeaconSens()) setVel2(-20,20);
	speedL=0;
	speedR=0;
	setVel2(speedL, speedR);
}

void turnWall(int state) {
	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	readAnalogSensors();                // Fill in "analogSensors" structure
	while(analogSensors.obstSensFront<30){
		
		speedL=30*state;
		speedR=-30*state;
		setVel2(speedL, speedR);
		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
	}

	waitTick40ms();                    // Wait for next 40ms tick (sensor provides a new value each 40 ms)
 	readAnalogSensors();				// Fill in "analogSensors" structure
	printf("Dist_left=%03d, Dist_center=%03d, Dist_right=%03d, Bat_voltage=%03d\n", 
       analogSensors.obstSensLeft,
       analogSensors.obstSensFront, 
       analogSensors.obstSensRight, 
       analogSensors.batteryVoltage);

	if(state==-1) {
		while(1) {
			if(checkCollision()) break;
			printf("Dist_left=%03d, Dist_center=%03d, Dist_right=%03d, Bat_voltage=%03d\n", 
		       analogSensors.obstSensLeft,
		       analogSensors.obstSensFront, 
		       analogSensors.obstSensRight, 
		       analogSensors.batteryVoltage);
			if(analogSensors.obstSensRight>30 && analogSensors.obstSensRight<40) {
				setVel2(50,10);
			}
			else if(analogSensors.obstSensRight<20) {
				setVel2(10,50);
			}
			else if(analogSensors.obstSensRight>40) {
				turnCorner(1);
				getBacon();
				break;
			}
			else {
				speedL=30;
				speedR=30;
				setVel2(speedL, speedR);
			}
		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
		}
	}
	if(state==1) {
			while(1) {
			if(checkCollision()) break;
			printf("Dist_left=%03d, Dist_center=%03d, Dist_right=%03d, Bat_voltage=%03d\n", 
		       analogSensors.obstSensLeft,
		       analogSensors.obstSensFront, 
		       analogSensors.obstSensRight, 
		       analogSensors.batteryVoltage);
			if(analogSensors.obstSensLeft>30 && analogSensors.obstSensLeft<40) {
				setVel2(30,70);
			}
			else if(analogSensors.obstSensLeft<20) {
				setVel2(70,30);
			}
			else if(analogSensors.obstSensLeft>40) {
				turnCorner(-1);
				getBacon();
				break;
			}
			else {
				speedL=30;
				speedR=30;
				setVel2(speedL, speedR);
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
        printf("%f, %f\n",dr, dl);
        if(dr>=25 && dl>=25) {
     		speedL=30;
			speedR=30;
			setVel2(speedL, speedR);
        }

        else {
         	if(dl<25) {
         		turnWall(1);
         	}
         	else if(dr<25) {
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

