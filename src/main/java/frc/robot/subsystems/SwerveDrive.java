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
  ArrayList<Wheel> frontWheelGroup;
  ArrayList<Wheel> backWheelGroup;

  public SwerveDrive(Wheel... driveTrain) {
    this.driveTrain = new ArrayList<Wheel>(Arrays.asList(driveTrain));
  }

  public SwerveDrive(Wheel frontRight, Wheel backRight, Wheel frontLeft, Wheel backLeft) {
    frontWheelGroup = new ArrayList<Wheel>(Arrays.asList(frontRight, frontLeft));
    backWheelGroup = new ArrayList<Wheel>(Arrays.asList(backRight, backLeft));
  }
  
  public void moveWIPAbs(double rightJoystickX, double rightJoystickY, double leftJoystickX, double leftJoystickY) {
    double speed = Math.sqrt((rightJoystickX * rightJoystickX) + (rightJoystickY * rightJoystickY));
    driveTrain.forEach(wheel -> {
      wheel.setSpeed(speed);
      wheel.setTargetAngle(angle(rightJoystickX, rightJoystickY));
    });
  }

  /**
   * right joystick distance from center controls speed
   * right joystick angle controls angle of the wheels
   * left joystick x axis rotates robot but the robot always keeps
   * "forward" as the same direction no matter which way it faces.
   */
  public void moveHeadless() {

  }

  public double angle(double joystickX, double joystickY) {
    return Math.atan2(joystickY, joystickX) * 180 / Math.PI;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
