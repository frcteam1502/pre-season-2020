/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.DrivetrainCommand;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {

  CANSparkMax frontRight, backRight, frontLeft, backLeft;

  /**
   * Creates a drivetrain with four motors using mecanum wheels.
   * @param fr Front right motor
   * @param br Back right motor
   * @param fl Front left motor
   * @param bl Back left motor
   */
  public Drivetrain(CANSparkMax fr, CANSparkMax br, CANSparkMax fl, CANSparkMax bl) {
    frontRight = fr;
    backRight = br;
    frontLeft = fl;
    backLeft = bl;
  }

  /**
   * @param speed  The forward/backward power - positive goes forward and negative
   *               goes backward
   * @param strafe The left/right power - positive moves right and negative moves
   *               left
   * @param turn   The turning power - positive turns right and negative turns
   *               left
   */
  public void drive(double speed, double strafe, double turn) {
    double fr = speed + strafe - turn;
    double br = speed - strafe - turn;
    double fl = speed - strafe + turn;
    double bl = speed + strafe + turn;
    double max = Collections.max(Arrays.asList(fr, br, fl, bl));
    double min = Collections.min(Arrays.asList(fr, br, fl, bl));
    double maxAbs = Math.max(max, -min);
    if (maxAbs > 1) {
      double ratio = 1 / maxAbs;
      fr *= ratio;
      br *= ratio;
      fl *= ratio;
      bl *= ratio;
    }
    frontRight.set(fr);
    backRight.set(br);
    frontLeft.set(fl);
    backLeft.set(bl);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DrivetrainCommand());
  }
}
