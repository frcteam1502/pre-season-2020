/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.DPadButton.Direction;
import frc.robot.commands.LidarSubsystem;
import frc.robot.commands.LinearSlideCommand;
import frc.robot.commands.RoboticArmMoveByCommand;
import frc.robot.subsystems.LinearSlide.Level;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
  public Joystick leftJoystick = new Joystick(RobotMap.LEFT_JOYSTICK);
	public Joystick rightJoystick = new Joystick(RobotMap.RIGHT_JOYSTICK);
	public XboxController manipJoystick = new XboxController(RobotMap.MANIP_JOYSTICK);
	public Joystick newJoystick = new Joystick(RobotMap.NEW_JOYSTICK);
	
	Button dpUp = new DPadButton(manipJoystick, Direction.Up);
	Button dpLeft = new DPadButton(manipJoystick, Direction.Left);
	Button dpDown = new DPadButton(manipJoystick, Direction.Down);
	Button dpRight = new DPadButton(manipJoystick, Direction.Right);
	Button back = new JoystickButton(manipJoystick, 7);

	public OI() {
		back.whenPressed(new Command() {
			{requires(Robot.slide);}
			protected boolean isFinished() {
				return true;
			}
			protected void initialize() {
				Robot.slide.toggle();
			}
		});
		dpUp.whenPressed(new LidarSubsystem());
		dpDown.whenPressed(new RoboticArmMoveByCommand(0, -1));
		dpRight.whenPressed(new RoboticArmMoveByCommand(1, 0));
		dpLeft.whenPressed(new RoboticArmMoveByCommand(-1, 0));
		// change that
		dpDown.whenPressed(new LinearSlideCommand(Level.Bottom));
		dpUp.whenPressed(new LinearSlideCommand(Level.Top));
	}
}
