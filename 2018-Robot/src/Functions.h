/*
 * Container.H
 *
 *  Created on: Feb 6, 2015
 *      Author: Connor
 */

#ifndef SRC_FUNCTIONS_H_
#define SRC_FUNCTIONS_H_

float checkMax(float liftAxis ,int JamesHit ,bool topSwitch ,bool botSwitch ,float potVal ,bool manual);
bool downButton(bool downButton, bool botSwitch);
bool upButton(bool upButton, bool topSwitch);

float adjust(float value);
float adjustSmooth(float value, float pastValue);

float rightPower(float throttle, float turn);
float leftPower(float throttle, float turn);

float integrate(float current, float total, float setPoint, float constant, float izone, bool continuous);
float PIDify(float past, float current, float setPoint, float pC, float iE, float dC, bool continuous);

float NavXR(float throttle, float angleDif);
float NavXL(float throttle, float angleDif);

int toteLifter(bool toteChecker, int binCount);

#endif /* SRC_FUNCTIONS_H_ */
