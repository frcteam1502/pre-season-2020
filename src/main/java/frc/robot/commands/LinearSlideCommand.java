package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LinearSlide.Level;

public class LinearSlideCommand extends Command {

  Level place;
  public LinearSlideCommand(Level place) {
    this.place = place;
    requires(Robot.slide);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.slide.setPlace(place);
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
