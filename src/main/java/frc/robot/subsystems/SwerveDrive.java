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

  /*
    This move module combines the intuitive nature of the absolute one, the full 
    utility of the relative one, but now it seperates the front and back sets of
    tires to give added mobility
    it averages out the x and y of two joysticks to give a seemingly tank drive
    like drive train, but seperated differently
  */
  public void moveWIPAbs(double throttle, double rightJoystickX, double rightJoystickY, double leftJoystickX, double leftJoystickY) {
    frontWheelGroup.forEach(wheel -> {
      wheel.setSpeed(throttle);
      if (wheel.getTurnAngle() > angle(leftJoystickX, leftJoystickY)) wheel.setTurn(-1);
      else if (wheel.getTurnAngle() < angle(leftJoystickX, leftJoystickY)) wheel.setTurn(1);
    });
    backWheelGroup.forEach(wheel -> {
      wheel.setSpeed(throttle);
      if (wheel.getTurnAngle() > angle(rightJoystickX, rightJoystickY)) wheel.setTurn(-1);
      else if (wheel.getTurnAngle() < angle(rightJoystickX, rightJoystickY)) wheel.setTurn(1);
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
