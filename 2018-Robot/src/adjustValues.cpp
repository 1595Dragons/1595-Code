/*
 * adjustValues.cpp
 *
 *  Created on: Feb 6, 2015
 *      Author: Connor
 */
#include "Functions.h"
#include "Math.h"
//adds deadbands
float adjust(float value){
	if (fabs(value)<.2) { value = 0;}

	if (value >= 0.9) {
		value = 1.0;
	}
	else if (value <= -0.9) {
		value = -1.0;
	}
	else if(value <0.9 && value>0.2)
	{
		value=(value-0.2)/0.7;
	}
	else if(value >-0.9 && value<-0.2)
	{
		value=(value+0.2)/0.7;
	}

	return(value);
}

float adjustSmooth(float value, float pastValue){

	if (fabs(value)>.2) {	}
	else {
		value = 0;
	}
	if (value >= 0.9) {
		value = 1.0;
	}
	else if (value <= -0.9) {
		value = -1.0;
	}
	if(value-pastValue > .1){
		if(value >= 0){
			value = pastValue+.05;
		}
	}
	if(value-pastValue < -.1){
		if(value <= 0){
			value = pastValue - .05;
		}
	}

	return(value);
}
