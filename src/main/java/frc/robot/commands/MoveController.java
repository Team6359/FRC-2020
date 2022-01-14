/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;

public class MoveController extends CommandBase {
  /**
   * Creates a new MoveController.
   */

  public MoveController() {
	  addRequirements(Robot.driveTrain);
  }

  public void execute() {
    	
		@SuppressWarnings("unused")
		boolean up, down, a, b, x, y, lB, rB, back, start, lClick, rClick, l3, lB2, rB2;
		@SuppressWarnings("unused")
		double lX, lY, rX, rY, lT, rT;
		int DPad, DPad2;
		boolean start2;

		double lT2, rT2;
		double lY2, rY2;

		boolean a2, b2, x2, y2;
		

		lX = OI.controller1.getRawAxis(0);
		lY = OI.controller1.getRawAxis(1);
		rX = OI.controller1.getRawAxis(4);
		rY = OI.controller1.getRawAxis(5);
		lT = OI.controller1.getRawAxis(2);
		rT = OI.controller1.getRawAxis(3);

		lY2 = OI.controller2.getRawAxis(1);
		rY2 = OI.controller2.getRawAxis(5);
		

		a = OI.controller1.getRawButton(1);
		b = OI.controller1.getRawButton(2);
		x = OI.controller1.getRawButton(3);
		y = OI.controller1.getRawButton(4);
		lB = OI.controller1.getRawButton(5);
		rB = OI.controller1.getRawButton(6);
		back = OI.controller1.getRawButton(7);
		start = OI.controller1.getRawButton(8);
		lClick = OI.controller1.getRawButton(9);
		rClick = OI.controller1.getRawButton(10);
		DPad = OI.controller1.getPOV();
		DPad2 = OI.controller2.getPOV();
		
		up = OI.controller1.getRawButton(10);
		down = OI.controller1.getRawButton(9);

		rT2 = OI.controller2.getRawAxis(3);
		lT2 = OI.controller2.getRawAxis(2);

		a2 = OI.controller2.getRawButton(1);
		b2 = OI.controller2.getRawButton(2);
		x2 = OI.controller2.getRawButton(3);
		y2 = OI.controller2.getRawButton(4);

		lB2 = OI.controller2.getRawButton(5);
		rB2 = OI.controller2.getRawButton(6);


		start2 = OI.controller2.getRawButton(8);

		Robot.driveTrain.drive(lY, rX, DPad, lClick, rClick, b);
		Robot.intake.runIntake(a, rB, rT- lT, lB, DPad, y, x, start, back);

  }


  public boolean isFinished() {
    return false;
  }
}
