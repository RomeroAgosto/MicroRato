#include "mr32.h"
#include <math.h>

void rotateRel_basic(int speed, double deltaAngle);
void turn_left();
void turn_right();
void half_turn();

int main(void){
	
	initPIC32(); //initializa o software do microrato
	closedLoopControl(true);
	setVel2(0, 0);
	while(1){
		while(!startButton());
		setVel2(0,0);

      enableObstSens();
      do{
         waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
         wait(1);
         readAnalogSensors();                // Fill in "analogSensors" structure
         if(analogSensors.obstSensFront<40){
            if(analogSensors.obstSensLeft>40 && analogSensors.obstSensRight<40){
               turn_left();
            }
            else if(analogSensors.obstSensFront<40 && analogSensors.obstSensLeft<40){
               turn_right();

            }
         }
         else{
            setVel2(30,30);
         }
         printf("Dist_left=%03d, Dist_center=%03d, Dist_right=%03d, Bat_voltage=%03d ", 
               analogSensors.obstSensLeft,
               analogSensors.obstSensFront, 
               analogSensors.obstSensRight, 
               analogSensors.batteryVoltage);
         printf("\n");
         /*if(analogSensors.obstSensFront<40 && analogSensors.obstSensLeft<40){
            turn_right();
         }
         if(){

         }*/
      }while(!stopButton());
      disableObstSens();
      setVel2(0, 0);
	}

}
void turn_right(){
      rotateRel_basic(100, (M_PI/2)*0.94); 
}

void turn_left(){
      rotateRel_basic(100, -(M_PI/2)*0.94); 
}
void half_turn(){
      rotateRel_basic(100, M_PI*0.94); 
}
/*void calibration_movement(){

}*/


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

