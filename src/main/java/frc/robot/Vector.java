package frc.robot;

import java.util.function.UnaryOperator;

public class Vector {
  public double x;
  public double y;

  public Vector(double x, double y) {
    this.x = x;
    this.y = y;
  }
  
  /**
   * Constructs a new vector from angle and magnitude
   * @param angle the angle in degrees
   * @param magnitude the magnitude (length) of the vector
   */
  public static Vector fromAngleAndMagnitude(double angle, double magnitude) {
    return new Vector(
      magnitude * Math.cos(Math.toRadians(angle)),
      magnitude * Math.sin(Math.toRadians(angle))
    );
  }

  public static double subtractAngles(double target, double current) {
    UnaryOperator<Double> constrainAngleToPositive = a -> {
      a = 360 - (-a % 360);
      a = a % 360;
      return a;
    };
    double absoluteDifference = constrainAngleToPositive.apply(target - current);
    return absoluteDifference > 180 ? absoluteDifference - 360 : absoluteDifference;
  }

  /**
   * Rotate a vector in Cartesian space.
   *
   * @param angle angle in degrees by which to rotate vector counter-clockwise.
   */
  public Vector rotate(double angle) {
    double cosA = Math.cos(Math.toRadians(angle));
    double sinA = Math.sin(Math.toRadians(angle));
    double[] out = new double[2];
    out[0] = x * cosA - y * sinA;
    out[1] = x * sinA + y * cosA;
    return new Vector(out[0], out[1]);
  }

  public double angle() {
    return Math.toDegrees(Math.atan2(y, x));
  }

  /**s
   * Returns dot product of this vector with argument.
   *
   * @param vec Vector with which to perform dot product.
   */
  public double dot(Vector vec) {
    return x * vec.x + y * vec.y;
  }

  /**
   * Returns magnitude of vector.
   */
  public double magnitude() {
    return Math.sqrt(x * x + y * y);
  }

  /**
   * Returns scalar projection of this vector onto argument.
   *
   * @param vec Vector onto which to project this vector.
   */
  public double scalarProject(Vector vec) {
    return dot(vec) / vec.magnitude();
  }

  public Vector normalize() {
    return Vector.fromAngleAndMagnitude(angle(), 1);
  }

  public Vector add(Vector other) {
    return new Vector(x + other.x, y + other.y);
  }

  public Vector multiply(double scalar) {
    return new Vector(x * scalar, y * scalar);
  }
}
