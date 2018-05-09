/**
 * \file
 * \brief example_mazerunner.c (MazeRunner - basic example)
 */

// ****************************************************************************
// EXAMPLE_MAZERUNNER.C
//
// To compile this example: pcompile example_mazerunner.c mr32.c
// ****************************************************************************
//
#include "mr32.h"
#include <math.h>

void rotateRel_basic(int speed, double deltaAngle);

int main(void)
{
   int groundSensor;

   initPIC32();
   closedLoopControl( true );
   setVel2(0, 0);

   printf("MazeRunner example\n\n\n");

   while(1)
   {
      printf("Press start to continue\n");
      while(!startButton());
      setVel2(30, 30);
      wait(10);
      setVel2(0, 0);

      enableObstSens();
      int i;
      //for(i=0;i<40;i++) {

         rotateRel_basic(50, (M_PI / 2) );
      //}
      do
      {
         waitTick40ms();                     // Wait for next 40ms tick (sensor provides a new value each 40 ms)
         readAnalogSensors();				      // Fill in "analogSensors" structure
         groundSensor = readLineSensors(0);	// Read ground sensor
         printf("Dist_left=%03d, Dist_center=%03d, Dist_right=%03d, Bat_voltage=%03d, Ground_sens=", 
               analogSensors.obstSensLeft,
               analogSensors.obstSensFront, 
               analogSensors.obstSensRight, 
               analogSensors.batteryVoltage);

         printInt(groundSensor, 2 | 5 << 16);	// System call
         printf("\n");
      } while(!stopButton());
      disableObstSens(); // Save battery charge
      rotateRel_basic(50, (-M_PI / 2) );
      setVel2(-30, -30);
      wait(10);
      setVel2(0, 0);
   }
   return 0;
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

