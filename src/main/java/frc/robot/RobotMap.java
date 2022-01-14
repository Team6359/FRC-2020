/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static final int R1 = 2;
	public static final int R2 = 3;
	public static final int L1 = 0;
	public static final int L2 = 1;

	public static final int intake1 = 6;
	public static final int intake2 = 7;
	public static final int intakeTop = 12;
	public static final int lScrew = 8;
	public static final int rScrew = 9;

	public static final int lBack = 4;
	public static final int rBack = 5;
	

	public static final int intakeSolenoid1 = 0;
	public static final int intakeSolenoid2 = 1;
	
	public static final int lScrewEnc1 = 6;
	public static final int lScrewEnc2 = 7;
	public static final int rScrewEnc1 = 8;
	public static final int rScrewEnc2 = 9;
	
	public static final int lBackEnc1 = 4;
	public static final int lBackEnc2 = 5;
	public static final int rBackEnc1 = 2;
	public static final int rBackEnc2 = 3;

	public static final int intakeLimit = 10;
	
	public static final int leftBackLimit = 1;
	public static final int rightBackLimit = 0;

	public static final int liftMotor = 11;

	// ROBOT CONSTANTS
	
	public static final float intakeSpeedBottom = 0.25f;
	public static final float intakeSpeedTop = 0.25f;
	public static final float shootSpeedBottom = 1f;
	public static final float shootSpeedTop = 1f;
	public static final float intakeBackSpeed = 0.6f;

	public static final int acceleration = 75; //Larger is slower

	public static final float liftSpeedUp = 0f;
	public static final float liftSpeedDown = 0.05f;
	

	public static final int liftSolenoid1 = 2;
	public static final int liftSolenoid2 = 3;

	public static final int fly1 = 12;
	public static final int fly2 = 7;
	
	public static final int frisSol1 = 0;
	public static final int frisSol2 = 1;

	public static final float flyWheelSpeed = 1f;

}
