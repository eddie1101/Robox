package org.usfirst.frc.team176.robot;

public class Pid {
	double kP = 0;
	double kI = 0;
	double kD = 0;
	double errorSum;
	double lastError;
	
	public void resetError(){
		errorSum = 0.0;
		lastError = 0.0;
	}
	
	public Pid(double p, double i, double d){
		kP = p;
		kI = i;
		kD = d;
		resetError();
	}
	public void setPid(double p, double i, double d){
		kP = p;
		kD = d;
		kI = i;
		resetError();
		
	}

	double pidUpdate(double goal, double currentValue)
	{
		double error = goal - currentValue;
		double p = kP * error;
		errorSum += error;
		double i = kI * errorSum;
		double dError = error - lastError;
		double d = kD * dError;
		lastError = error;
		return p + i + d;
	}
}
