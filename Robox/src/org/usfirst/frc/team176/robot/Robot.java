/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team176.robot;

import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Robot extends TimedRobot {
	
	private enum State{
		INIT_HOME,
		IDLE,
		HOME_X_INIT,
		HOME_X,
		HOME_Y_INIT,
		HOME_Y,
		HOME_B_INIT,
		HOME_B,
		HOME_A_INIT,
		HOME_A,
		MANUAL;
	}
	
	private final double dt = 0.2;
	
	private final double kA = 0.00000;
	private final double kV = 0.00000;
	private final double kP = 0.00100;
	
	private double mPosition = 0;
	private double mVelocity = 0;
	private double mAcceleration = 0;
	
	private double mTimestamp = 0;
	
	private double m_maxAccel;
	private double m_cruseVeloc;
	private double m_decelB;
	private double m_velocityIntegral;
	private double m_displacement;
	private double m_profileDuration;
	
	private double neededPosition = 0;
	private double neededVelocity = 0;
	private double neededAcceleration = 0;
	
	private final double kHX = 100;
	private final double kHY = 2200;
	private final double kHB = 800;
	private final double kHA = 1600;
	
	private State mState;
	private State mNextState = State.INIT_HOME;
	private NetworkTable mNetworkTable;
	private byte[] m_incomingBuffer;
	private I2C mi2c;
	private TalonSRX talon;
	private Joystick joystick;
	private VictorSP victor;
	private DigitalInput reverse;
	private DigitalInput forward;
	private edu.wpi.first.wpilibj.Encoder encoder;
	private Pid pid;

	static double clamp(double n, double lower, double upper) {
		if(n < lower) {
			return lower;
		} else if(n > upper) {
			return upper;
		}
		return n;
	}
	@Override
	public void robotInit() {
//		mNetworkTable = new NetworkTable();
		mState = State.INIT_HOME;
		m_incomingBuffer = new byte[2];
		mi2c = new I2C(I2C.Port.kMXP, 0x62);
		talon = new TalonSRX(30);
		victor = new VictorSP(2);
		joystick = new Joystick(0);
		reverse = new DigitalInput(0);
		forward = new DigitalInput(1);
		encoder = new edu.wpi.first.wpilibj.Encoder(2,3);
		pid = new Pid(0.01,0.0,0.0);		
	}

	
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		System.out.print(forward.get());
		System.out.println(reverse.get());
	}


	@Override
	public void teleopInit() {
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		if(joystick.getRawButton(8)){
			mNextState = State.MANUAL;
		}
		if(joystick.getRawButton(7)){
			mNextState = State.INIT_HOME;
		}
		if(joystick.getRawButton(3)){
			mNextState = State.HOME_X_INIT;
		}
		if(joystick.getRawButton(1)){
			mNextState = State.HOME_A_INIT;
		}
		if(joystick.getRawButton(2)){
			mNextState = State.HOME_B_INIT;
		}
		if(joystick.getRawButton(4)){
			mNextState = State.HOME_Y_INIT;
		}
		
		double lastPos = mPosition;
		mPosition = encoder.get();
		double lastVel = mVelocity;
		mVelocity = mPosition - lastPos;
		mAcceleration = mVelocity - lastVel;
		
		double demand = 0;
//		demand = pid.pidUpdate(1200, encoder.get());
//		demand = clamp(demand, -0.3, 0.3);

		System.out.println("Position: " + encoder.get());
		System.out.println("State: " + mState);
		System.out.println("NextState: " + mNextState);
		System.out.println("Demand " + demand);
		
		System.out.println("NeededPosition: " + neededPosition);
		System.out.println("NeededVelocity: " + neededVelocity);
		System.out.println("NeededAcceleration: " + neededAcceleration);
		
		System.out.println("dt: " + dt);
		
		switch(mState){
			case INIT_HOME:
//				System.out.println(reverse.get());
				if(reverse.get() == false){
					demand = -0.25;
				}
				else{
					victor.set(0);
					encoder.reset();
					mNextState = State.IDLE;
				}
				break;
			case IDLE:
				demand = 0;
				break;
			case HOME_X_INIT:
				ProfileInit(mPosition - kHX, 100, 50);
				mTimestamp = System.currentTimeMillis();
				mNextState = State.HOME_X;
				break;
			case HOME_X:
				GetPointForTime(System.currentTimeMillis() - mTimestamp);
				demand = (kA * neededAcceleration) + (kV * neededVelocity) + (kP * (neededPosition - mPosition));
				break;
			case HOME_Y_INIT:
				ProfileInit(mPosition - kHY, 100, 50);
				mTimestamp = System.currentTimeMillis();
				mNextState = State.HOME_Y;
				break;
			case HOME_Y:
				GetPointForTime(System.currentTimeMillis() - mTimestamp);
				demand = (kA * neededAcceleration) + (kV * neededVelocity) + (kP * (neededPosition - mPosition));
				break;
			case HOME_B_INIT:
				ProfileInit(mPosition - kHB, 100, 50);
				mTimestamp = System.currentTimeMillis();
				mNextState = State.HOME_B;
				break;
			case HOME_B:
				GetPointForTime(System.currentTimeMillis() - mTimestamp);
				demand = (kA * neededAcceleration) + (kV * neededVelocity) + (kP * (neededPosition - mPosition));
				break;
			case HOME_A_INIT:
				ProfileInit(mPosition - kHA, 100, 50);
				mTimestamp = System.currentTimeMillis();
				mNextState = State.HOME_A;
				break;
			case HOME_A:
				GetPointForTime(System.currentTimeMillis() - mTimestamp);
				demand = (kA * neededAcceleration) + (kV * neededVelocity) + (kP * (neededPosition - mPosition));
				break;
			case MANUAL:
				demand = joystick.getRawAxis(4) * 0.25;
				break;
				
			default:
				break;
		}
		if(mState != mNextState){
			mState = mNextState;
		}
		
		if(forward.get() && demand > 0){
			demand = 0;
		}
		if(reverse.get() && demand < 0){
			demand = 0;
		}
		
		victor.set(demand);
		
		System.out.println("Position: " + encoder.get());
		
		mi2c.write(0x00, 0x04);
		try {
			Thread.sleep(40);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		bzero(m_incomingBuffer);
		mi2c.read(0x8f, 2, m_incomingBuffer);
		float incomingDistance = (int)((m_incomingBuffer[0] << 8) + m_incomingBuffer[1]);
		
		System.out.println("Distance: " + incomingDistance + "(cm)\n");
		
	
	}
	
	public void bzero(byte[] bytes){
		for(int i = 0; i < bytes.length; i++){
			bytes[i] = 0;
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	public void autonomousInit(){
		
	}
	public void autonomousPeriodic(){
		if(reverse.get() == false){
		victor.set(-0.1);
		}
		else{
			victor.set(0);
			encoder.reset();
		}
		System.out.println(encoder.get());
	}
	
	void ProfileInit(double displacement, double maxAccel, double cruseVeloc) {
		m_displacement = displacement;
		m_maxAccel = maxAccel;
		m_cruseVeloc = cruseVeloc;

		m_velocityIntegral = 0;

		double normalTriangleWidth = (cruseVeloc / maxAccel);
		double totalTriangleArea = (cruseVeloc / maxAccel) * cruseVeloc;

		if(totalTriangleArea >= displacement) {
			//The rectangle width will be negative - this profile will be a triangle

			m_decelB = maxAccel * 2 * Math.sqrt(displacement / maxAccel);
		} else {
			//The rectangle with positive - this profile will be a rectangle

			double rectangleArea = displacement - totalTriangleArea;
			double rectangleWidth = rectangleArea / cruseVeloc;
			double decelXInt = 2 * normalTriangleWidth + rectangleWidth;
			m_profileDuration = decelXInt;
			m_decelB = maxAccel * decelXInt;
		}
	}

	void GetPointForTime(double time) {
		double velocity = Math.max(Math.min(m_maxAccel * time, Math.min(m_cruseVeloc, -1 * m_maxAccel * time + m_decelB)), 0.0) * Math.signum(m_displacement);

		m_velocityIntegral += velocity * dt; //Assumes that this is called every 10ms
		neededPosition = m_velocityIntegral;
		neededVelocity = velocity;
		if(time > m_profileDuration || velocity == m_cruseVeloc) {
			neededAcceleration = 0;
		} else {
			if(time < (m_profileDuration / 2)) {
				neededAcceleration = m_maxAccel;
			} else {
				neededAcceleration = -1 * m_maxAccel;
			}
		}
	}
}
	
