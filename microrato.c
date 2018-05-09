#include "mr32.h"
#include <math.h>

void rotateRel_basic(int speed, double deltaAngle);
void getBacon();
void goTo();
void turnWall(int state);
void turnCorner(int state);
int checkCollision();
void moveOutTheWay(int dist);
void inBed();
void getBed();


int volta, timeToEat;
char flagTurn;
char cornerFlag=0;
int baconRotate=0.1;
int groundSensor;
double x, y, t;

int main(void){
	initPIC32(); //initializa o software do microrato
	closedLoopControl(true);
	char count=0;
	setVel2(0, 0);
	while(1){
		while(!startButton());
		wait (1);
      	enableObstSens();
    	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
	    setServoPos(0);
	    leds(0);

	    getRobotPos(&x,&y,&t);
	    volta=0;
	    do{
	    	getBacon();
	    	while(!groundSensor) {
	    		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    		readAnalogSensors();                // Fill in "analogSensors" structure
	    		goTo();
         		groundSensor = readLineSensors(0);	// Read ground sensor	
	    	}
	    	leds(15);
			setVel2(0, 0);
			volta=1;
			getBed();
			do{
				goTo();
				inBed();
			}while(!timeToEat);
			setVel2(0,0);
			while(1) {
				if(count) {
					leds(0);
					delay(700);
					count++;
				}
				else {
					leds(15);
					delay(700);
					count--;
				}
			}
	    }while(!stopButton());
	}

}

void inBed() {
	getRobotPos(&x,&y,&t);
	if(fabs(x)<100 && fabs(y)<100) timeToEat=1;
	else timeToEat=0;
}

void getBed() {

	setVel2(0,0);
	getRobotPos(&x,&y,&t);
	double teta;
	double tmp, h;
	tmp=pow(x,2);
    h=pow(y,2);
    h+=tmp;
    h=sqrt(h);
    h=fabs(h);
	teta=asin(x/h);
	if (x>0) {
		teta=-teta;
		teta+=M_PI;
	}
	/*printf("X=%03f, Y=%03f, H=%03f, T=%03f\n", 
           x,
           y, 
           h, 
           t);
	printf("%03f",teta);*/
	teta=-teta;
	teta-=M_PI/2;
	teta-=t;
	rotateRel_basic(20, teta);
}

int checkCollision() {
	waitTick40ms();                    // Wait for next 40ms tick (sensor provides a new value each 40 ms)
 	readAnalogSensors();				// Fill in "analogSensors" structure
 	if(analogSensors.obstSensFront<10) return 1;
 	else return 0;

}


void getBacon() {
	setVel2(-20,20);
	while(!readBeaconSens()) ;
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
	rotateRel_basic(20, flagTurn*state*M_PI/20);
	moveOutTheWay(270);
	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	readAnalogSensors();                // Fill in "analogSensors" structure
	if(checkCollision()) return;
	if(state==-1) {
		setVel2(0,50);
		wait(8);
		setVel2(0,0);
		if(analogSensors.obstSensFront>30) {
			moveOutTheWay(270);
		}
	}
	else {
		rotateRel_basic(-20, M_PI/2);
		if(analogSensors.obstSensFront>30) {
			moveOutTheWay(270);
		}
	}
}

void turnWall(int state) {
	waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	readAnalogSensors();                // Fill in "analogSensors" structure
	while(analogSensors.obstSensFront<30){
		
		setVel2(40*state, -40*state);
		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
	}

	waitTick40ms();                    // Wait for next 40ms tick (sensor provides a new value each 40 ms)
 	readAnalogSensors();				// Fill in "analogSensors" structure

	if(state==-1) {
		while(1) {
			if(checkCollision()) break;
			if(analogSensors.obstSensRight>25 && analogSensors.obstSensRight<50) {
				cornerFlag=-1;
				flagTurn=0;
				setVel2(40,20);
			}
			else if(analogSensors.obstSensRight<20) {
				flagTurn=1;
				cornerFlag=0;
				setVel2(20,40);
			}
			else if(analogSensors.obstSensRight>55) {
				if(cornerFlag){
					turnCorner(1);
					if(!volta) getBacon();
					else getBed();
					break;
				}
				else  {
					rotateRel_basic(20, -M_PI/20);
					flagTurn=1;
					cornerFlag=1;
				}
			}
			else if(analogSensors.obstSensFront<25){
				cornerFlag=0;
				setVel2(-20,20);
				delay(350);
				if(volta) {
					inBed();
					break;
				}
			}
			else {
				cornerFlag=0;
				setVel2(40, 40);
			}
		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
		}
	}
	if(state==1) {
			while(1) {
			if(checkCollision()) break;
			if(analogSensors.obstSensLeft>25 && analogSensors.obstSensLeft<50) {
				flagTurn=0;
				cornerFlag=0;
				setVel2(20,40);
			}
			else if(analogSensors.obstSensLeft<20) {
				flagTurn=1;
				cornerFlag=0;
				setVel2(40,20);
			}
			else if(analogSensors.obstSensLeft>50) {
				if(cornerFlag){
					turnCorner(-1);
					if(!volta) getBacon();
					else getBed();
					break;
				}
				else  {
					rotateRel_basic(20, M_PI/20);
					flagTurn=1;
					cornerFlag=1;
				}
			}
			else if(analogSensors.obstSensFront<25){
				cornerFlag=0;
				setVel2(20,-20);
				delay(350);
				if(volta) {
					inBed();
					break;
				}
			}
			else {
				cornerFlag=0;
				setVel2(40, 40);
			}
		waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
	    readAnalogSensors();                // Fill in "analogSensors" structure
		}
	}
}

void goTo() {
	double tmp, dr, dl;
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
    /*printf("Dist_left=%03d, Dist_center=%03d, Dist_right=%03d, Bat_voltage=%03d, Dr=%03f, Dl=%03f\n", 
               analogSensors.obstSensLeft,
               analogSensors.obstSensFront, 
               analogSensors.obstSensRight, 
               analogSensors.batteryVoltage,
               dr,
               dl);*/
    if(dr>=30 && dl>=30) {
		setVel2(40, 40);
    }

    else {
     	if(analogSensors.obstSensLeft<30) {
     		turnWall(1);
     	}
     	else if(analogSensors.obstSensRight<30) {
     		turnWall(-1);
     	}
     	else rotateRel_basic(20, M_PI);
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