/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX R1, R2, L1, L2;
  private TalonSRX liftMotor;
  private Solenoid liftSolenoid1, liftSolenoid2;
  private Encoder liftEncoder;
  

  public DriveSubsystem(){
    R1 = new TalonSRX(RobotMap.R1); 
    R2 = new TalonSRX(RobotMap.R2);
    L1 = new TalonSRX(RobotMap.L1);
    L2 = new TalonSRX(RobotMap.L2);
    
    liftMotor = new TalonSRX(RobotMap.liftMotor);
    liftEncoder = new Encoder(11, 12);
    liftSolenoid1 = new Solenoid(RobotMap.liftSolenoid1);
    liftSolenoid2 = new Solenoid(RobotMap.liftSolenoid2);


  }

	private double curSpeedLeft = 0;
	private double curSpeedRight = 0;

  double leftSpeed, rightSpeed, x, y, error;
  double curSpeedDown = 0;

  boolean slowMode = false;

  boolean dbB = false;
  int aiming = 0;

  double liftSpeed = 0;
  public void drive(double leftYAxis, double rightXAxis, int DPad, boolean lStick, boolean rStick, boolean b){

    slowMode = lStick ? true : rStick ? false : slowMode;

    y = Math.pow(leftYAxis, 3);
    x = Math.pow(-rightXAxis, 3);		

    if (Math.abs(leftSpeed - rightSpeed) > 0.1) {
      leftSpeed *= 0.5;
      rightSpeed *= 0.5;
    }
    error = 0;
    leftSpeed = (y + x) + error;
    rightSpeed = (y - x) - error;
    if ((curSpeedLeft < leftSpeed && curSpeedLeft >= 0) || (curSpeedLeft > leftSpeed && curSpeedLeft <= 0))
      curSpeedLeft += leftSpeed / 75;
    if ((curSpeedRight < rightSpeed && curSpeedRight >= 0) || (curSpeedRight > rightSpeed && curSpeedRight <= 0))
      curSpeedRight += rightSpeed / 75;
    if ((leftSpeed < curSpeedLeft && curSpeedLeft >= 0) || (leftSpeed > curSpeedLeft && curSpeedLeft <= 0))
      curSpeedLeft -= leftSpeed / 75;
    if ((rightSpeed < curSpeedRight && curSpeedRight >= 0) || (rightSpeed > curSpeedRight && curSpeedRight <= 0))
      curSpeedRight -= rightSpeed / 75;
    
    
    if (Math.abs(leftSpeed) < 0.1) {
      curSpeedLeft = 0;
    }
    if (Math.abs(rightSpeed) < 0.1) {
      curSpeedRight = 0;
    }

    SmartDashboard.putBoolean("Turning", Math.abs(leftSpeed + rightSpeed) > 0.1);

    System.out.println("Cur Left Speed: " + curSpeedLeft + "Cur Right Speed: " + curSpeedLeft);
    System.out.println("Left Speed: " + leftSpeed + " Right Speed: " + rightSpeed);
    
    if (slowMode){
      rightSpeed *= 0.5;
      leftSpeed *= 0.5;
      double newR = leftSpeed;
      leftSpeed = rightSpeed;
      rightSpeed = newR;
    }

    DPad = -1;

    if (DPad == 0 && curSpeedDown < 0.6 && liftEncoder.getRaw() < 42300){
      curSpeedDown += RobotMap.liftSpeedDown;
    } else if (DPad != 0 || liftEncoder.getRaw() >= 42300){
      curSpeedDown = 0;
    }

    if (DPad == 90 && liftSpeed > -1){
      liftSpeed -= RobotMap.liftSpeedDown;
    } else if (DPad != 90){
      liftSpeed = 0;
    }

    

    liftMotor.set(ControlMode.PercentOutput, DPad == 180 ? RobotMap.liftSpeedUp : (DPad == 0 ? curSpeedDown : DPad == 90 ? liftSpeed : 0));

    if (DPad != 180 && DPad != 0 && DPad != 90){
      liftSolenoid1.set(true);
      liftSolenoid2.set(false);
    } else {
      liftSolenoid1.set(false);
      liftSolenoid2.set(true);
    }

    SmartDashboard.putNumber("Lift Encoder", liftEncoder.getRaw());

    NetworkTableEntry tx = Robot.limelight.getEntry("tx");
    NetworkTableEntry ty = Robot.limelight.getEntry("ty");
    NetworkTableEntry ta = Robot.limelight.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);


    SmartDashboard.putNumber("Limelightx", x);
    SmartDashboard.putNumber("Limelighty", y);
    SmartDashboard.putNumber("LimelightArea", area);

    if (b && !dbB){
      aiming = aiming == 0 ? 1 : 0;
      Robot.limelight.getEntry("ledMode").setNumber(aiming);
      Robot.limelight.getEntry("camMode").setNumber(aiming);
      dbB = true;
    }

    if (!b){
      dbB = false;
    }
    
    double kPX = 0.018;

    double xError = kPX * x;

    SmartDashboard.putNumber("xError", xError);

    R1.set(ControlMode.PercentOutput, rightSpeed + xError);
    R2.set(ControlMode.PercentOutput, rightSpeed + xError);
    L1.set(ControlMode.PercentOutput, -leftSpeed + xError);
    L2.set(ControlMode.PercentOutput, -leftSpeed + xError);

}

}
