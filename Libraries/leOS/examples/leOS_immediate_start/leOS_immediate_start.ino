/*
This is a simple sketch to demonstrate the use of leOS, a
<l>ittle <e>mbedded <O>perating <S>ystem

leOS is not a complete OS nor an RTOS as you usually know but it's a simple
scheduler to schedule little works in background, so that you can forget about
them.

This sketch will add 2 tasks that have to be run at different intervals. One of
them will start as soon as it has been added to the scheduler.
To test the sketch, connect 2 LEDs (with their corresponding resistors)
to pins D7 and D8.

More info on the scheduler and the methods can be found into the README file.

Written by Leonardo Miliani <www.leonardomiliani.com>
	
This code and the leOS library are free software; you can redistribute 
and/or modify them under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 3.0 of the License,
or (at your option) any later version.

The leOS library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 

*/

#include "leOS.h" //include the scheduler
leOS myOS; //create a new istance

//variables to manage the LEDs
const byte LED1 = 7;
const byte LED2 = 8;
byte LED1status = 1;
byte LED2status = 1;


//setup routine
void setup() {
    myOS.begin(); //initialize the scheduler
    //pins as output
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    //add the tasks
    myOS.addTask(flashLed1, 2000, SCHEDULED_IMMEDIATESTART); //this starts once it has been added
    myOS.addTask(flashLed2, 1000);
}


//main loop - it's empty
void loop() {}


//this task will flash the 1st LED
void flashLed1() {
    digitalWrite(LED1, LED1status);
    LED1status ^= 1;
}


//this task will flash the 2nd LED
void flashLed2() {
    digitalWrite(LED2, LED2status);
    LED2status ^= 1;
}