package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.PIDController;
import frc.robot.commands.LinearSlideMoveCommand;

public class LinearSlide extends Subsystem {

  private CANSparkMax leftMotor, rightMotor;
  private CANEncoder leftEnc, rightEnc;
  private PIDController leftPID, rightPID;

  private final int BOTTOM_CARGO = 1000;
  private final int BOTTOM_HATCH = 1100;
  
  private final int TOP_CARGO = 2000;
  private final int TOP_HATCH = 2100;

  private final Position CARGO = new Position(BOTTOM_CARGO, TOP_CARGO);
  private final Position HATCH = new Position(BOTTOM_HATCH, TOP_HATCH);
  private Position currentType = CARGO;
  private Level currentLevel = null;
  
  public enum Level {
    Bottom, Top
  }
  
  public LinearSlide(CANSparkMax leftMotor, CANSparkMax rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    leftEnc = new CANEncoder(leftMotor);
    rightEnc = new CANEncoder(rightMotor);
    leftPID = new PIDController(1e-5, 1e-8, 1e-2);
    rightPID = new PIDController(1e-5, 1e-8, 1e-2);
  }

  public class Position {
    int bottom;
    int top;
    Position(int bottom, int top) {
      this.bottom = bottom;
      this.top = top;
    }
  }

  public void move() {
    int encVal = currentLevel == Level.Bottom ? currentType.bottom : currentType.top;
    leftPID.input(encVal - leftEnc.getPosition());
    rightPID.input(encVal - rightEnc.getPosition());
    leftMotor.set(leftPID.getCorrection());
    rightMotor.set(rightPID.getCorrection());
  }

  public void setLevel(Level place) {
    currentLevel = place;
  }
  
  public void toggle() {
    currentType = currentType != CARGO ? CARGO : HATCH;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new LinearSlideMoveCommand());
  }
}
