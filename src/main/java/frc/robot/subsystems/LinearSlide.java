package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.command.Subsystem;

public class LinearSlide extends Subsystem {

  static CANSparkMax left, right;
  static CANEncoder leftEnc, rightEnc;

  private final int BOTTOM_CARGO = 1000;
  private final int BOTTOM_HATCH = 1100;
  
  private final int TOP_CARGO = 2000;
  private final int TOP_HATCH = 2100;

  Position cargo = new Position(BOTTOM_CARGO, TOP_CARGO);
  Position hatch = new Position(BOTTOM_HATCH, TOP_HATCH);
  Position currentType = cargo;
  
  public enum Level {
    bottom, top
  }
  
  public LinearSlide(CANSparkMax leftMotor, CANSparkMax rightMotor) {
    left = leftMotor;
    right = rightMotor;
    leftEnc = new CANEncoder(left);
    rightEnc = new CANEncoder(right);
  }

  public class Position {
    int bottom;
    int top;
    Position(int bottom, int top) {
      this.bottom = bottom;
      this.top = top;
    }
  }

  public void moveTo(Level place) {
    int encVal = place == Level.bottom ? currentType.bottom : currentType.top;
    if (leftEnc.getPosition() < encVal && rightEnc.getPosition() < encVal) {
      left.set(1.0);
      right.set(-1.0);
    }
    else if (leftEnc.getPosition() > encVal && rightEnc.getPosition() > encVal) {
      left.set(-1.0);
      right.set(1.0);
    }
  }

  public void toggle() {
    currentType = currentType != cargo ? cargo : hatch;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
