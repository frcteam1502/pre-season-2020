/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;

/**
 * Add your docs here.
 */
public class RoboticArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private int armLength, foreArmLength;
  private int min, max;
  CANSparkMax armMotor, forearmMotor;

  public RoboticArm(int armLength, int forearmLength, CANSparkMax armMotor, CANSparkMax forearmMotor) {
    this.armLength = armLength;
    this.foreArmLength = forearmLength;
    this.armMotor = armMotor;
    this.forearmMotor = forearmMotor;
    min = armLength - forearmLength;
    max = armLength + forearmLength;
  }

  public double[] run(double[] pos) throws ArithmeticException {
    double distance = Math.sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
    double distMultiplier = 1;
    if (distance > max)
        distMultiplier = max / distance;
    if (distance < min)
        distMultiplier = min / distance;
    distance *= distMultiplier;
    double[] endPosition = { pos[0] * distMultiplier, pos[1] * distMultiplier };

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
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
