/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  
  // Drivetrain sparks. using sparks for encoders
  public static final CANSparkMax FRONT_RIGHT = new CANSparkMax(0, MotorType.kBrushed);
  public static final CANSparkMax BACK_RIGHT = new CANSparkMax(1, MotorType.kBrushed);
  public static final CANSparkMax FRONT_LEFT = new CANSparkMax(2, MotorType.kBrushed);
  public static final CANSparkMax BACK_LEFT = new CANSparkMax(3, MotorType.kBrushed);

  // Swerve Drivetrain sparks. using wheel to condense into one reference
  public static final Wheel FRONT_RIGHT_SWERVE = new Wheel(new CANSparkMax(4, MotorType.kBrushed), new CANSparkMax(5, MotorType.kBrushed), new Vector(1, -1));
  public static final Wheel BACK_RIGHT_SWERVE = new Wheel(new CANSparkMax(6, MotorType.kBrushed), new CANSparkMax(7, MotorType.kBrushed), new Vector(-1, -1));
  public static final Wheel FRONT_LEFT_SWERVE = new Wheel(new CANSparkMax(8, MotorType.kBrushed), new CANSparkMax(9, MotorType.kBrushed), new Vector(1, 1));
  public static final Wheel BACK_LEFT_SWERVE = new Wheel(new CANSparkMax(10, MotorType.kBrushed), new CANSparkMax(11, MotorType.kBrushed), new Vector(-1, 1));

  // Controllers
  public static final int LEFT_JOYSTICK = 0;
  public static final int RIGHT_JOYSTICK = 1;
  public static final int MANIP_JOYSTICK = 2;

  // Just random test stuff, to be commented out here, oi, and robot
  public static final CANSparkMax LINEAR_SLIDE_LEFT = new CANSparkMax(12, MotorType.kBrushless);
  public static final CANSparkMax LINEAR_SLIDE_RIGHT = new CANSparkMax(13, MotorType.kBrushless);

  public static final CANSparkMax FOREARM_MOTOR = new CANSparkMax(14, MotorType.kBrushed);
  public static final CANSparkMax ARM_MOTOR = new CANSparkMax(15, MotorType.kBrushed);

  // Sensors
  public static final ADXRS450_Gyro GYRO = new ADXRS450_Gyro(Port.kOnboardCS0);
  public static final UsbCamera CAMERA = new UsbCamera("USB Camera 0", 0);

}
