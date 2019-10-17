/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.RoboticArm;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot {

  public static Drivetrain drivetrain;
  public static LinearSlide slide;
  public static OI m_oi;
  public static SwerveDrive swerveDrive;
  public static RoboticArm arm;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    drivetrain = new Drivetrain(RobotMap.FRONT_RIGHT, RobotMap.BACK_RIGHT, RobotMap.FRONT_LEFT, RobotMap.BACK_LEFT);
    // test stuff
    slide = new LinearSlide(RobotMap.LINEAR_SLIDE_LEFT, RobotMap.LINEAR_SLIDE_RIGHT);
    swerveDrive = new SwerveDrive(RobotMap.FRONT_RIGHT_SWERVE, RobotMap.BACK_RIGHT_SWERVE, RobotMap.FRONT_LEFT_SWERVE, RobotMap.BACK_LEFT_SWERVE);
    arm = new RoboticArm(10, 5, RobotMap.ARM_MOTOR, RobotMap.FOREARM_MOTOR);
    m_oi = new OI();

    SmartDashboard.putData("Auto mode", m_chooser);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }
}
