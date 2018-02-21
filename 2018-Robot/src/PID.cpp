/*
 * Calculate.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: Connor
 */
#include "Functions.h"

float integrate(float current, float total, float setPoint, float constant, float izone, bool continuous){
	float curError = current - setPoint;
	if(curError > izone || curError < -izone) { return(0); }
	if(continuous){
		if(curError > 180){
			curError = curError - 360;
		}
		else if(curError < -180){
			curError = curError+360;
		}
	}
	curError = curError * constant;
	total = total+curError*constant;
	return(total);
}

float PIDify(float past, float current, float setPoint, float pC, float iE, float dC, bool continuous){
	float curError
	,pastError
	,pE ,dE ,tE;

	curError = current-setPoint;
	pastError = past-setPoint;
	if(continuous){
		if(curError > 180){
			curError = curError - 360;
		}
		else if(curError < -180){
			curError = curError + 360;
		}
		if(current - pastError > 180){
			current = current - 360;
		}
		if(current - pastError < -180){
			pastError = pastError + 360;
		}
	}

	pE = curError * pC;
	dE = (curError-pastError)* dC;
	tE = pE + iE + dE;
	return(tE);
}

