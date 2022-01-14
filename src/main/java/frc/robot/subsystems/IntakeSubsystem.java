/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private Solenoid solenoid1, solenoid2, shootSol1, shootSol2;
  private VictorSPX intake1, intake2, intakeTop, lScrew, rScrew, lBack, rBack, fly1, fly2;
  private Encoder lScrewEnc, rScrewEnc, lBackEnc, rBackEnc;
  private DigitalInput rollerLimitSwitch, limitLeft, limitRight;

  private boolean lBOn = false;
  private boolean lBDebouce = false;

  public IntakeSubsystem() {
   // solenoid1 = new Solenoid(RobotMap.intakeSolenoid1);
   // solenoid2 = new Solenoid(RobotMap.intakeSolenoid2);
    intake1 = new VictorSPX(RobotMap.intake1);
    //intake2 = new VictorSPX(RobotMap.intake2);
    //intakeTop = new VictorSPX(RobotMap.intakeTop);
    shootSol1 = new Solenoid(RobotMap.frisSol1);
    shootSol2 = new Solenoid(RobotMap.frisSol2);
    lScrew = new VictorSPX(RobotMap.lScrew);
    rScrew = new VictorSPX(RobotMap.rScrew);
    lBack = new VictorSPX(RobotMap.lBack);
    rBack = new VictorSPX(RobotMap.rBack);
    fly1 = new VictorSPX(RobotMap.fly1);
    fly2 = new VictorSPX(RobotMap.fly2);

    lScrewEnc = new Encoder(RobotMap.lScrewEnc1, RobotMap.lScrewEnc2);
    rScrewEnc = new Encoder(RobotMap.rScrewEnc1, RobotMap.rScrewEnc2);

    rollerLimitSwitch = new DigitalInput(RobotMap.intakeLimit);
    limitLeft = new DigitalInput(RobotMap.leftBackLimit);
    limitRight = new DigitalInput(RobotMap.rightBackLimit);

    lBackEnc = new Encoder(RobotMap.lBackEnc1, RobotMap.lBackEnc2);
    rBackEnc = new Encoder(RobotMap.rBackEnc1, RobotMap.rBackEnc2);

  
    rBack.setInverted(true);
  }

  double kP = 0.00006;

  double startTime = 0;
  boolean outtaking = false;
  double spinupTimeTop = 750; //Delay between outtake motors and back moving
  double spinupTimeBottom = 1500; //Delay between outtake motors and back moving

  boolean seen = false;

  float shootSpeed = 1f;
  boolean rightChange = false;
  boolean leftChange = false;

  public void runIntake(boolean a, boolean rB, double triggerAggregate, boolean lB, int pov, boolean y, boolean x, boolean start, boolean back){
  /*
    if (rB){
      extend();
      if (!lB){
        intake1.set(ControlMode.PercentOutput, RobotMap.intakeSpeedBottom); //Intake direction
        intake2.set(ControlMode.PercentOutput, RobotMap.intakeSpeedBottom);
        intakeTop.set(ControlMode.PercentOutput, -1 *  RobotMap.intakeSpeedTop);
      } else {
        intake1.set(ControlMode.PercentOutput, -1 * RobotMap.intakeSpeedBottom); //Outake direction whlie extended
        intake2.set(ControlMode.PercentOutput, -1 * RobotMap.intakeSpeedBottom);
        intakeTop.set(ControlMode.PercentOutput, RobotMap.intakeSpeedTop);
      }
    } else if (a && !outtaking){
      startTime = System.currentTimeMillis();
      intake1.set(ControlMode.PercentOutput, 0); //Outake direction whlie retracted
      intake2.set(ControlMode.PercentOutput, 0);
      intakeTop.set(ControlMode.PercentOutput, RobotMap.shootSpeedTop);
      outtaking = true;
    }
  */
    if (!rB){
     // retract();
    }

    // if (lB){
    //   intake1.set(ControlMode.PercentOutput, -1 * RobotMap.intakeSpeedBottom); //Intake direction
    //   intake2.set(ControlMode.PercentOutput, -1 * RobotMap.intakeSpeedBottom);
    //   intakeTop.set(ControlMode.PercentOutput, RobotMap.intakeSpeedTop);
    // }

    if (lB){
      if (!lBDebouce){
        lBOn = !lBOn;
        lBDebouce = true;
      }
    } else {
      lBDebouce = false;
    }
    
    if (!a){
      outtaking = false;
    }

    if (pov == 90){
      if (!rightChange){
        rightChange = true;
        if (shootSpeed < 1f){
          shootSpeed += 0.1;
        }
      }
    } else {
      rightChange = false;
    }

    if (pov == 270){
      if (!leftChange){
        leftChange = true;
        if (shootSpeed > 0){
          shootSpeed -= 0.1;
        }
      }
    } else {
      leftChange = false;
    }

    if (lBOn){
      fly1.set(ControlMode.PercentOutput, shootSpeed);
      fly2.set(ControlMode.PercentOutput, shootSpeed);
    } else {
      fly1.set(ControlMode.PercentOutput, 0);
      fly2.set(ControlMode.PercentOutput, 0);
    }

    if (rB){
      shootSol1.set(false);
      shootSol2.set(true);
    } else {
      shootSol1.set(true);
      shootSol2.set(false);
    }
    
  


    
    double errorBack = ((rBackEnc.getRaw() * -1) - lBackEnc.getRaw());
    double kPBack = 0.005;

    // if (outtaking){
    //   if (System.currentTimeMillis() - startTime >= spinupTimeTop){
    //     intake1.set(ControlMode.PercentOutput, RobotMap.shootSpeedBottom); //Outake direction whlie retracted
    //     intake2.set(ControlMode.PercentOutput, RobotMap.shootSpeedBottom);
    //   }
    //   if (System.currentTimeMillis() - startTime >= spinupTimeBottom){
    //     intake1.set(ControlMode.PercentOutput, RobotMap.shootSpeedBottom); //Outake direction whlie retracted
    //     intake2.set(ControlMode.PercentOutput, RobotMap.shootSpeedBottom);
    //     intakeTop.set(ControlMode.PercentOutput, RobotMap.shootSpeedTop);
    //     if (lBackEnc.getRaw() < 2300){
    //       lBack.set(ControlMode.PercentOutput, RobotMap.intakeBackSpeed + errorBack * kPBack);
    //       rBack.set(ControlMode.PercentOutput, RobotMap.intakeBackSpeed);
    //     } else {
    //       lBack.set(ControlMode.PercentOutput, 0);
    //       rBack.set(ControlMode.PercentOutput, 0);
    //     }
    //   }
    // } else {
    //   lBack.set(ControlMode.PercentOutput, 0);
    //   rBack.set(ControlMode.PercentOutput, 0);
    //   if (!rB && !lB){
    //     intake1.set(ControlMode.PercentOutput, 0);
    //     intake2.set(ControlMode.PercentOutput, 0);
    //     intakeTop.set(ControlMode.PercentOutput, 0);
    //   }
    // }

   

    if (!outtaking){

      if (lBackEnc.getRaw() < 1000){
        if (limitLeft.get())
          lBack.set(ControlMode.PercentOutput, -0.15 + errorBack * kPBack);
        if (limitRight.get())
          rBack.set(ControlMode.PercentOutput, -0.15);
      } else {
        if (limitLeft.get())
          lBack.set(ControlMode.PercentOutput, -0.3 + errorBack * kPBack);
        if (limitRight.get())
          rBack.set(ControlMode.PercentOutput, -0.3);
      }

      
    }

    NetworkTableEntry ty = Robot.limelight.getEntry("ty");
    NetworkTableEntry ta = Robot.limelight.getEntry("ta");
    
    double kPA = 0.25;
   
    double yErr = ty.getDouble(0.0) + (14);
    if (ty.getDouble(0.0) == 0){
      yErr -= (14);
    }

    
   
    double degrees = (rScrewEnc.getRaw() / 1000) * .081444 + 4.3;
    double distance = 68 / Math.tan(Math.toRadians(degrees - 13));



    double kPD = 0.0005;
    double kPY = 0.04;
    double yError = yErr * kPY - ((distance - 120) * kPD);

    if (yErr == 0){
      yError = 0;
    }

    boolean down = triggerAggregate < 0;
    boolean up = triggerAggregate > 0;
    boolean liftLimit = rollerLimitSwitch.get();

    double error = (rScrewEnc.getRaw() - lScrewEnc.getRaw());

    double rEnc = rScrewEnc.getRaw();


    boolean normalGood = ((down && rEnc > -17875) || (up && rEnc < 500000)) && Math.abs(triggerAggregate) > 0.2;
    boolean xModeGood = x && ((!liftLimit && down) || up);
    if (normalGood || xModeGood){
      if ((liftLimit && down) || (rEnc > 450000 && up)){
        lScrew.set(ControlMode.PercentOutput, triggerAggregate / 5);
        rScrew.set(ControlMode.PercentOutput, triggerAggregate / 5 + error * kP);
      } else if ((!liftLimit && down) || up){
        lScrew.set(ControlMode.PercentOutput, triggerAggregate);
        rScrew.set(ControlMode.PercentOutput, triggerAggregate + error * kP);
      } else if (x){
        lScrew.set(ControlMode.PercentOutput, triggerAggregate);
        rScrew.set(ControlMode.PercentOutput, error * kP);
    } 
  }
    else {
      if ((yError < 0 && rEnc > -17865) || (yError > 0 && rEnc < 500000)){
        lScrew.set(ControlMode.PercentOutput, yError);
        rScrew.set(ControlMode.PercentOutput, error * kP - yError);
      }
      else {
        lScrew.set(ControlMode.PercentOutput, 0);
        rScrew.set(ControlMode.PercentOutput, error * kP);
      }
    }

    SmartDashboard.putNumber("YERROR", yError);

    if (start){
      lScrew.set(ControlMode.PercentOutput, 0.7);
      lScrewEnc.reset();
      rScrewEnc.reset();
    }

    if (back){
      lScrew.set(ControlMode.PercentOutput, -0.7);
      lScrewEnc.reset();
      rScrewEnc.reset();
    }
  
    if (!limitLeft.get())
      lBackEnc.reset();
    if (!limitRight.get())
      rBackEnc.reset();

    if (liftLimit && !seen){
      lScrewEnc.reset();
      rScrewEnc.reset();
      seen = true;
    }

    if (!liftLimit){
      seen = false;
    }
    

    SmartDashboard.putNumber("TriggerAggregate", triggerAggregate);

    SmartDashboard.putNumber("Left Encoder", lScrewEnc.getRaw());
    SmartDashboard.putNumber("Right Encoder", rScrewEnc.getRaw());
    SmartDashboard.putNumber("Error", rScrewEnc.getRaw() - lScrewEnc.getRaw());
    SmartDashboard.putNumber("Error #", rScrewEnc.getRaw() - lScrewEnc.getRaw());
    SmartDashboard.putNumber("Error * kP", (rScrewEnc.getRaw() - lScrewEnc.getRaw()) * kP);
    SmartDashboard.putBoolean("Roller Limit sSwitch", rollerLimitSwitch.get());

    SmartDashboard.putBoolean("Back Limit Switch Left", limitLeft.get());
    SmartDashboard.putBoolean("Back Limit Switch Right", limitRight.get());


    SmartDashboard.putNumber("Left Back Encoder", lBackEnc.getRaw());
    SmartDashboard.putNumber("Right Back Encoder", rBackEnc.getRaw());

    SmartDashboard.putNumber("Error Back", errorBack);
    SmartDashboard.putNumber("Error Back * kP", errorBack * kPBack);
    SmartDashboard.putNumber("Degrees", degrees);
    SmartDashboard.putNumber("Distance", distance);

    
    
  }
  

  // public void extend(){
  //   solenoid1.set(false);
  //   solenoid2.set(true);
  // }

  // public void retract(){
  //   solenoid1.set(true);
  //   solenoid2.set(false);
  // }

  public void periodic() {
  }

}
