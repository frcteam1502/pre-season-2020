/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

public class Wheel {

  CANSparkMax turnMotor, moveMotor;
  CANEncoder turnEncoder, moveEncoder;
  PIDController turnPID, movePID; // <-- sure we need movePID?
  // maybe in auton
  private double targetAngle;
  private double targetSpeed;
  public Vector turnRightVector;

  Wheel(CANSparkMax turnMotor, CANSparkMax moveMotor, Vector turnRightVector) {
    this.turnMotor = turnMotor;
    turnEncoder = new CANEncoder(turnMotor);
    turnPID = new PIDController(1e-5, 1e-8, 1e-2);
    this.moveMotor = moveMotor;
    moveEncoder = new CANEncoder(moveMotor);
    this.turnRightVector = turnRightVector.normalize();
  }

  /**
   * Sets the target velocity of the wheel
   */
  public void setTargetVelocity(Vector velocity) {
    setTargetAngle(velocity.angle());
    setTargetSpeed(velocity.magnitude());
  }

  /**
   * This function sets the wheel's target angle and speed based on its target
   * vector. It should be called by the robot every tick.
   */
  public void applyVectorMovement() {
    double currentAngle = getTurnAngle();
    double angleDifference = Vector.subtractAngles(targetAngle, currentAngle);
    turnPID.input(angleDifference);
    moveMotor.set(targetSpeed);
    turnMotor.set(turnPID.getCorrection());
  }

  public void setTargetAngle(double angle) {
    targetAngle = angle;
  }

  public void setTargetSpeed(double speed) {
    if (speed < -1 || speed > 1) {
      speed = 0;
      throw new RuntimeException("Magnitude of velocity vector is too high.");
    }
    targetSpeed = speed;
  }

  private double getTurnAngle() {
    return (turnEncoder.getPosition() / (double) turnEncoder.getCPR() * 360);
  }
}
