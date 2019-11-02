/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PIDController;
import frc.robot.Vector;
import frc.robot.commands.RoboticArmMoveCommand;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

public class RoboticArm extends Subsystem {

  private int armLength, forearmLength;
  private int minDistance, maxDistance;
  private CANSparkMax armMotor, forearmMotor;
  private CANEncoder armEnc, forearmEnc;
  private PIDController armPID, forearmPID;
  public Vector targetPosition = new Vector(0, 0);

  // public RoboticArm(int armLength, int forearmLength, Talon armMotor, Talon forearmMotor) {
  //   this.armLength = armLength;
  //   this.forearmLength = forearmLength;
  //   this.armMotor = armMotor;
  //   this.forearmMotor = forearmMotor;
  //   armEnc = new CANEncoder(armMotor);
  //   armPID = new PIDController(1e-5, 1e-8, 1e-2);
  //   forearmPID = new PIDController(1e-5, 1e-8, 1e-2);
  //   forearmEnc = new CANEncoder(forearmMotor);
  //   minDistance = armLength - forearmLength;
  //   maxDistance = armLength + forearmLength;
  // }

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

  /**
   * @deprecated in progress.
   * <br></br>
   * when arm is straight, arm angle = 0, forearm angle = 90
   * @see com.revrobotics.CANEncoder#setPosition(double)
   */
  public void initRoboticArm() {
    armEnc.setPosition(0);
    forearmEnc.setPosition(angleToEncoderVal(forearmEnc, -90));
  }

  /**
   * used for an arm with 360 degree rotation on the forearm
   * @param endPoint
   * @return angles flipped over the y axis
   */
  private double[] getIdealArmAngles(Vector endPoint) {
    double[] angles;
    if (endPoint.x < 0) {
      angles = getInvertedAngles(endPoint);
    }
    else angles = getAngles();
    return angles;
  }
  /**
   * @param endPoint
   * @return angle from x axis to arm - non-inverted angle of the arm
   *  <li> the opposite angle of the non-inverted forearm angle
   */
  private double[] getInvertedAngles(Vector endPoint) {
    double[] nonInverted = getAngles();
    double angleToArm = endPoint.angle();
    return new double[] {2 * angleToArm - nonInverted[0], 180 - nonInverted[1]};
  }
  
  /**
   * @param x change of the endpoint on the x axis
   * @param y change of the endpoint on the y axis
   */
  public void changeEndPoint(int x, int y) {
    targetPosition = targetPosition.add(new Vector(x, y));
  }

  /**
   * @param x new x value of the endpoint
   * @param y new y value of the endpoint
   */
  public void setEndPoint(int x, int y) {
    targetPosition = new Vector(x, y);
  }

  private double getMotorAngle(CANEncoder motor) {
    return (motor.getPosition() % motor.getCPR() / (double) motor.getCPR() * 360);
  }
  
  private double angleToEncoderVal(CANEncoder motor, double angle) {
    return angle / 360 * motor.getCPR();
  }

  /**
   * <ul>
   *  <li> gets motor angle of forearm and arm
   *  <li> compares with idea angles
   *  <li> uses PID Controller to get motor speed
   * </ul>
   */
  public void run() {
    double armEncAngle = getMotorAngle(armEnc);
    double forearmEncAngle = getMotorAngle(forearmEnc);
    double[] angles = getIdealArmAngles(targetPosition);
    armPID.input(Vector.subtractAngles(angles[0], armEncAngle));
    forearmPID.input(Vector.subtractAngles(angles[1], forearmEncAngle));
    armMotor.set(armPID.getCorrection());
    forearmMotor.set(forearmPID.getCorrection());
  }

  /**
   * @throws ArithmeticException if differenceFromEndpointToElbow is undefined
   * 
   * @returns Angle of the arm in relation to the robot and angle of the
   * forearm in relation to the arm
   */
  public double[] getAngles() {
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
    return new double[] { firstAngle, secondAngle };
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new RoboticArmMoveCommand());
  }
}
