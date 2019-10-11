package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LinearSlideMoveCommand extends Command {

  public LinearSlideMoveCommand() {
    requires(Robot.slide);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.slide.move();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
