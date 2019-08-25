package frc.robot;

public class RoboticArm {

  private int armLength, foreArmLength;
  private int min, max;

  public RoboticArm(int armLength, int foreArmLength) {
    this.armLength = armLength;
    this.foreArmLength = foreArmLength;
    min = armLength - foreArmLength;
    max = armLength + foreArmLength;
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
    // double[] elbow = { armLength * Math.cos(firstAngle), armLength *
    // Math.sin(firstAngle) };
    System.out.print(firstAngle + ", " + secondAngle);
    double[] angles = { firstAngle, secondAngle };
    return angles;
  }
}
