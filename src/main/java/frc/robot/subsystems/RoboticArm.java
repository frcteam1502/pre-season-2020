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
import frc.robot.commands.RoboticArmMoveCommand;

import java.util.function.UnaryOperator;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

public class RoboticArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private int armLength, foreArmLength;
  private int min, max;
  private CANSparkMax armMotor, forearmMotor;
  private CANEncoder armEnc, forearmEnc;
  private PIDController armPID, forearmPID;
  private Vector endPoint = new Vector(0,0);

  public RoboticArm(int armLength, int forearmLength, CANSparkMax armMotor, CANSparkMax forearmMotor) {
    this.armLength = armLength;
    this.foreArmLength = forearmLength;
    this.armMotor = armMotor;
    armEnc = new CANEncoder(armMotor);
    armPID = new PIDController(1e-5, 1e-8, 1e-2);
    forearmPID = new PIDController(1e-5, 1e-8, 1e-2);
    this.forearmMotor = forearmMotor;
    forearmEnc = new CANEncoder(forearmMotor);
    min = armLength - forearmLength;
    max = armLength + forearmLength;
  }

  /**
   * @deprecated in progress.
   * <br></br>
   * {@code when arm is straight, arm angle = 0, forearm angle = 90};
   * @see com.revrobotics.CANEncoder
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
    else angles = getAngles(endPoint);
    return angles;
  }
  /**
   * @param endPoint
   * @return angle from x axis to arm - non-inverted angle of the arm
   *  <li> the opposite angle of the non-inverted forearm angle
   */
  private double[] getInvertedAngles(Vector endPoint) {
    double[] nonInverted = getAngles(endPoint);
    double angleToArm = endPoint.angle();
    return new double[] {2 * angleToArm - nonInverted[0], 180 - nonInverted[1]};
  }
  
  /**
   * @param x change of the endpoint on the x axis
   * @param y change of the endpoint on the y axis
   */
  public void changeEndPoint(int x, int y) {
    endPoint = endPoint.add(new Vector(x, y));
  }
  /**
   * @param x new x value of the endpoint
   * @param y new y value of the endpoint
   */
  public void setEndPoint(int x, int y) {
    endPoint = new Vector(x, y);
  }

  private double getMotorAngle(CANEncoder motor) {
    return (motor.getPosition() % 360 / (double) motor.getCPR() * 360);
  }
  
  private double angleToEncoderVal(CANEncoder motor, double angle) {
    return angle / 360 * motor.getCPR();
  }

  /**
   * does <br>
   * <ul>
   *  <li> gets motor angle of forearm and arm
   *  <li> compares with idea angles
   *  <li> uses PID Controller to get motor speed
   * </ul>
   */
  public void run() {
    double armEncAngle = getMotorAngle(armEnc);
    double forearmEncAngle = getMotorAngle(forearmEnc);
    double[] angles = getIdealArmAngles(endPoint);
    armPID.input(subtractAngles(angles[0], armEncAngle));
    forearmPID.input(subtractAngles(angles[1], forearmEncAngle));
    armMotor.set(armPID.getCorrection());
    forearmMotor.set(forearmPID.getCorrection());
  }

  /**
   * -180 =< val =< 180
   * @param target angle you are going to
   * @param current angle you are at
   * @return target - current
   */
  private double subtractAngles(double target, double current) {
    UnaryOperator<Double> constrainAngleToPositive = a -> {
        a = 360 -(-a % 360);
        a = a % 360;
        return a;
    };
    double absoluteDifference = constrainAngleToPositive.apply(target - current);
    return absoluteDifference > 180 ? absoluteDifference - 360 : absoluteDifference;
}

  /**
   * @param endPoint final destination of arm
   * @exception ArithmeticException a2 could be undefined
   * @return <b>angles</b>: Angle of the arm in relation to the robot and angle of the forearm in relation to the arm
   */
  private double[] getAngles(Vector endPoint) throws ArithmeticException {
    double distance = Math.sqrt(endPoint.x * endPoint.x + endPoint.y * endPoint.y);
    double distMultiplier = 1;
    if (distance > max)
        distMultiplier = max / distance;
    if (distance < min)
        distMultiplier = min / distance;
    distance *= distMultiplier;
    Vector endPosition = endPoint.multiply(distMultiplier);

    double a1 = Math.atan2(endPosition.y, endPosition.x);
    double secondAngle = Math.asin((endPosition.y * endPosition.y + endPosition.x * endPosition.x
            - armLength * armLength - foreArmLength * foreArmLength) / 2 / armLength / foreArmLength);
    double a2 = Math.asin(foreArmLength * Math.cos(secondAngle) / distance);
    double firstAngle = a1 + a2;
    // double[] elbow = { armLength * Math.cos(firstAngle), armLength * Math.sin(firstAngle) };
    System.out.print(firstAngle + ", " + secondAngle);
    double[] angles = { firstAngle, secondAngle };
    return angles;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new RoboticArmMoveCommand());
  }
}
