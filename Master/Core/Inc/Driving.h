/*
 * Driving.h
 *
 *  Created on: Nov 28, 2023
 *      Author: lucas
 */

#ifndef INC_DRIVING_H_
#define INC_DRIVING_H_
#include "Communication.h"


void Forward() {
    char message[20];  // Create a character array to hold the message

    int distance = 50  ;  // Create a character array to hold the message
    int speed =  100;
    // Format the message with the current LED value
    sprintf(message, "drvstr %d %d\r\n", distance, speed);

    // Send the message over UART
    SendUARTMessage(message );



}
void Right(){
    char message[20];  // Create a character array to hold the message

    int angle = 45  ;  // Create a character array to hold the message
    int speed =  100;
    // Format the message with the current LED value
    sprintf(message, "rotctr %d %d\r\n", angle, speed);

    // Send the message over UART
    SendUARTMessage(message);


}
void Left(){
    char message[20];  // Create a character array to hold the message

    int angle = -45  ;  // Create a character array to hold the message
    int speed =  100;
    // Format the message with the current LED value
    sprintf(message, "rotctr %d %d\r\n", angle, speed);

    // Send the message over UART
    SendUARTMessage(message);


}


#endif /* INC_DRIVING_H_ */
