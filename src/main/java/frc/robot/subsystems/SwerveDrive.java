/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Wheel;

public class SwerveDrive extends Subsystem {

  ArrayList<Wheel> driveTrain;

  public SwerveDrive(Wheel... driveTrain) {
    this.driveTrain = new ArrayList<Wheel>(Arrays.asList(driveTrain));
  }

  public void moveByRelative(double moveSpeed, double rotateSpeed) {
    driveTrain.forEach(x -> {
      x.moveMotor.set(moveSpeed);
      x.turnMotor.set(rotateSpeed); 
    });
  }

  public void moveByAngle(double speed, double xLen, double yLen) {
    driveTrain.forEach(x -> {
      x.moveMotor.set(speed);
      if (x.turnEncAngle() > angle(xLen, yLen)) x.turnMotor.set(-1);
      else if (x.turnEncAngle() < angle(xLen, yLen)) x.turnMotor.set(1);
    });
  }

  public double angle(double xLen, double yLen) {
    return Math.atan(yLen / xLen) * 180 / Math.PI;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
