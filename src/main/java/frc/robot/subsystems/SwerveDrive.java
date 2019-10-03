/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.stream.Collectors;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Vector;
import frc.robot.Wheel;

public class SwerveDrive extends Subsystem {

  Wheel[] wheels;

  public SwerveDrive(Wheel... wheels) {
    this.wheels = wheels;
  }

  /**
   * Right joystick distance from center controls speed,
   * right joystick angle controls angle of the wheels,
   * left joystick x axis rotates robot.
   */
  public void moveMode2(double x, double y, double turn) {
    Vector translationVector = new Vector(x, y);
    double largestSpeed = 0;
    ArrayList<Vector> wheelVelocities = new ArrayList<>();
    for (int i = 0; i < wheels.length; i++) {
      Vector rotationVector = wheels[i].turnRightVector.multiply(turn);
      Vector resultant = translationVector.add(rotationVector);
      largestSpeed = Math.min(resultant.magnitude(), largestSpeed);
      wheelVelocities.add(resultant);
    }
    if (largestSpeed > 1) {
      double scalar = 1 / largestSpeed;
      wheelVelocities = new ArrayList<>(wheelVelocities.stream().map(vel -> vel.multiply(scalar)).collect(Collectors.toList()));
    }
    for (int i = 0; i < wheels.length; i++) {
      wheels[i].setTargetVelocity(wheelVelocities.get(i));
    }
  }

  /**
   * Just like {@link frc.robot.subsystems.SwerveDrive#moveMode2}
   * but keeps the robot's heading away from the user at all times.
   */
  public void moveHeadless(double x, double y, double turn) {
    Vector moveVector = new Vector(x, y);
    moveVector = moveVector.rotate(-RobotMap.GYRO.getAngle());
    moveMode2(moveVector.x, moveVector.y, turn);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
