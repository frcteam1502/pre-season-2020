/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PIDController;
import frc.robot.Vector;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

/**
 * Add your docs here.
 */
public class RoboticArm extends Subsystem {

  private int armLength, forearmLength;
  private int minDistance, maxDistance;
  CANSparkMax armMotor, forearmMotor;
  CANEncoder armEnc, forearmEnc;
  PIDController armPID, forearmPID;
  public Vector targetPosition = new Vector(0, 0);

  public RoboticArm(int armLength, int forearmLength, CANSparkMax armMotor, CANSparkMax forearmMotor) {
    this.armLength = armLength;
    this.forearmLength = forearmLength;
    this.armMotor = armMotor;
    this.forearmMotor = forearmMotor;
    armEnc = new CANEncoder(armMotor);
    armPID = new PIDController(1e-5, 1e-8, 1e-2);
    forearmPID = new PIDController(1e-5, 1e-8, 1e-2);
    forearmEnc = new CANEncoder(forearmMotor);
    minDistance = armLength - forearmLength;
    maxDistance = armLength + forearmLength;
  }

  public void moveBy(Vector difference) {
    targetPosition = targetPosition.add(difference);
  }

  public void moveTo(Vector target) {
    targetPosition = target;
  }

  private double getMotorAngle(CANEncoder motor) {
    return (motor.getPosition() / (double) motor.getCPR() * 360);
  }

  public void run() {
    double armEncAngle = getMotorAngle(armEnc);
    double forearmEncAngle = getMotorAngle(forearmEnc);
    double[] angles = getAngles();
    armPID.input(Vector.subtractAngles(angles[0], armEncAngle));
    forearmPID.input(Vector.subtractAngles(angles[1], forearmEncAngle));
    armMotor.set(armPID.getCorrection());
    forearmMotor.set(forearmPID.getCorrection());
  }

  /*
   * @throws ArithmeticException if a2 is undefined
   * 
   * @returns angles Angle of the arm in relation to the robot and angle of the
   * forearm in relation to the arm
   **/
  private double[] getAngles() throws ArithmeticException {
    double distanceFromOrigin = targetPosition.magnitude();
    double distMultiplier = 1;
    if (distanceFromOrigin > maxDistance)
      distMultiplier = maxDistance / distanceFromOrigin;
    if (distanceFromOrigin < minDistance)
      distMultiplier = minDistance / distanceFromOrigin;
    distanceFromOrigin *= distMultiplier;
    Vector endPosition = targetPosition.multiply(distMultiplier);

    double endPointAngleFromOrigin = endPosition.angle();
    double secondAngle = Math.asin((Math.pow(endPosition.magnitude(), 2) - armLength * armLength - forearmLength * forearmLength) / 2 / armLength / forearmLength);
    double differenceFromEndpointToElbow = Math.asin(forearmLength * Math.cos(secondAngle) / distanceFromOrigin);
    double firstAngle = endPointAngleFromOrigin + differenceFromEndpointToElbow;
    System.out.println(firstAngle + ", " + secondAngle);
    return new double[]{ firstAngle, secondAngle };
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
