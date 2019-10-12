/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PIDController;
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
  private int[] endPoint = {0,0};

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
   * In progress.
   * hope is that it raises the arm to its max point on the y axis, and then sets
   * the encoder values to 0 from there.
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

  private double[] getAnglesAbs(int[] endPoint) {
    double[] angles;
    if (endPoint[0] < 0) {
      endPoint[0] = Math.abs(endPoint[0]);
      angles = getAngles(endPoint);
      angles[0] = Math.abs(angles[0] - 360);
      angles[1] = Math.abs(angles[1] - 360);
    }
    else angles = getAngles(endPoint);
    return angles;
  }
  
  public void changeEndPoint(int x, int y) {
    endPoint[0] += x;
    endPoint[1] += y;
  }

  public void setEndPoint(int x, int y) {
    endPoint[0] = x;
    endPoint[1] = y;
  }

  private double getMotorAngle(CANEncoder motor) {
    return (motor.getPosition() % 360 / (double) motor.getCPR() * 360);
  }
  
  private double angleToEncoderVal(CANEncoder motor, double angle) {
    return angle / 360 * motor.getCPR();
  }

  /**
   * gets correction of target angle - current angle
   * and sets motor speed
   */

  public void run() {
    double armEncAngle = getMotorAngle(armEnc);
    double forearmEncAngle = getMotorAngle(forearmEnc);
    double[] angles = getAnglesAbs(endPoint);
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
   * @param pos Position of both arm end points
   * @throws ArithmeticException a2 could be undefined
   * @returns angles Angle of the arm in relation to the robot and angle of the forearm in relation to the arm
   */
  private double[] getAngles(int[] endPoint) throws ArithmeticException {
    double distance = Math.sqrt(endPoint[0] * endPoint[0] + endPoint[1] * endPoint[1]);
    double distMultiplier = 1;
    if (distance > max)
        distMultiplier = max / distance;
    if (distance < min)
        distMultiplier = min / distance;
    distance *= distMultiplier;
    double[] endPosition = { endPoint[0] * distMultiplier, endPoint[1] * distMultiplier };

    double a1 = Math.atan2(endPosition[1], endPosition[0]);
    double secondAngle = Math.asin((endPosition[1] * endPosition[1] + endPosition[0] * endPosition[0]
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
