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
  
  // Controllers
	public static final int LEFT_JOYSTICK = 0;
	public static final int RIGHT_JOYSTICK = 1;
  public static final int MANIP_JOYSTICK = 2;
  
  // Peripherals
  public static final UsbCamera CAMERA = new UsbCamera("USB Camera 0", 0);

}
