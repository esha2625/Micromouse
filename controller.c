/*
 * controller.c
 *
 *  Created on: Sep 27, 2020
 *      Author: Tyler Price
 */

#include "main.h"
#include "controller.h"
#include "pid.h"
#include "irs.h"

/*
 * We recommend you implement this function so that move(1) will move your rat 1 cell forward.
 */
void move(int8_t n)
{
	/*
	 * For assignment 3.1: Don't worry about implementing this yet
	 * For assignment 3.2: This function should set the distance and angle goals appropriately for PID (hint: using the setGoal functions in pid.c)
	 * and wait until the error becomes sufficiently small and persistent before exiting. This function should NOT exit before then.
	 *
	 * HINT: Use a while loop to wait until the rat has moved the desired distance
	 *
	 * You should also call resetPID before exiting this function so your rat is ready for the next instruction.
	 */
	int16_t newgoal=0;
	int16_t midr_ir=2300;
	int16_t midl_ir=1270;
	setPIDGoalD(615*n);
	setPIDGoalA(newgoal);
	updatePID();
	int16_t right_ir=readRightIR();
	int16_t left_ir=readLeftIR();
	int16_t frontl_ir=readFrontLIR();
	int16_t frontr_ir=readFrontRIR();
	int16_t front_ir = (frontl_ir+frontr_ir)/2;
	int16_t div_ir=20;
	if (front_ir>2000)
	{
		newgoal= (frontl_ir-frontr_ir)/div_ir;
		setPIDGoalA(newgoal);
	}

	/*else if (left_ir>900 && right_ir> 900)
	{
		int16_t ir_errorL= left_ir-midl_ir;
		int16_t ir_errorR= midr_ir-right_ir;
		int16_t ir_error= ((ir_errorL+ir_errorR)/2)/div_ir;
		midl_ir *= (9/10);
		midl_ir += left_ir * (1/10);
		midr_ir *= (9/10);
		midr_ir += right_ir * (1/10);
		setPIDGoalA(ir_error);
	}*/
	else if (left_ir>900)
	{
		int16_t ir_error = (left_ir-midl_ir)/div_ir;
		setPIDGoalA(ir_error);
		if (left_ir> 1900)
		{
			setPIDGoalA(80);
		}
	}
	else if (right_ir>900)
	{
		int16_t ir_error = (midr_ir-right_ir)/div_ir;
		setPIDGoalA(ir_error);
		if (right_ir> 2500)
		{
			setPIDGoalA(-80);
		}
	}

	while (!(PIDdoneD()==1) || !(PIDdoneA()==1))
	{
		if (front_ir>2370)
		{
			overridedistance();
		}
	}
	resetPID();
	HAL_Delay(20);
}

void moveback(int8_t n)
{
	/*
	 * For assignment 3.1: Don't worry about implementing this yet
	 * For assignment 3.2: This function should set the distance and angle goals appropriately for PID (hint: using the setGoal functions in pid.c)
	 * and wait until the error becomes sufficiently small and persistent before exiting. This function should NOT exit before then.
	 *
	 * HINT: Use a while loop to wait until the rat has moved the desired distance
	 *
	 * You should also call resetPID before exiting this function so your rat is ready for the next instruction.
	 */
	setPIDGoalD((-35)*n);
	setPIDGoalA(0);
	updatePID();
	while (!(PIDdoneD()==1) || !(PIDdoneA()==1))
	{

	}
	resetPID();
	HAL_Delay(20);
}

/*
 * We recommend you implement this function so that turn(1) turns your rat 90 degrees in your positive rotation
 * direction and turn(-1) turns the other way.
 */
void turn(int8_t n)
{
	/*
	 * For assignment 3.1: Don't worry about implementing this yet
	 * For assignment 3.2: This function should set the distance and angle goals appropriately for PID (hint: using the setGoal functions in pid.c)
	 * and wait until the error becomes sufficiently small and persistent before exiting. This function should NOT exit before then.
	 *
	 * HINT: Use a while loop to wait until the turn is complete
	 *
	 * You should also call resetPID before exiting this function so your rat is ready for the next instruction.
	 */
	int16_t right_ir=readRightIR();
	int16_t left_ir=readLeftIR();
	int16_t front_ir=(readFrontRIR()+readFrontLIR())/2;
	/*if ((right_ir > 3000) && (n>0))
	{
		//turn the right motor backwards a bit
		correctturnL(1);
	}*/
	setPIDGoalA(566*n); //was 560
	setPIDGoalD(20); //was 20
	updatePID();
	while (!(PIDdoneA()==1)|| !(PIDdoneD()==1))
	{

	}
	resetPID();
	HAL_Delay(50);
}

void correctturnL(int8_t n)
{
	/*
	 * For assignment 3.1: Don't worry about implementing this yet
	 * For assignment 3.2: This function should set the distance and angle goals appropriately for PID (hint: using the setGoal functions in pid.c)
	 * and wait until the error becomes sufficiently small and persistent before exiting. This function should NOT exit before then.
	 *
	 * HINT: Use a while loop to wait until the turn is complete
	 *
	 * You should also call resetPID before exiting this function so your rat is ready for the next instruction.
	 */
	setPIDGoalA(5*n); //was 550
	setPIDGoalD(0);
	updatePID();
	while (!(PIDdoneA()==1)|| !(PIDdoneD()==1))
	{
		//updatePID();
	}
	resetPID();
}

void correctturnR(int8_t n)
{
	/*
	 * For assignment 3.1: Don't worry about implementing this yet
	 * For assignment 3.2: This function should set the distance and angle goals appropriately for PID (hint: using the setGoal functions in pid.c)
	 * and wait until the error becomes sufficiently small and persistent before exiting. This function should NOT exit before then.
	 *
	 * HINT: Use a while loop to wait until the turn is complete
	 *
	 * You should also call resetPID before exiting this function so your rat is ready for the next instruction.
	 */
	setPIDGoalA((-5)*n);
	setPIDGoalD(0);
	updatePID();
	while (!(PIDdoneA()==1)|| !(PIDdoneD()==1))
	{
		//updatePID();
	}
	resetPID();
}

