/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

/**
 * Add your docs here.
 */
public class Wheel {
    
    public CANSparkMax turnMotor, moveMotor;
    public CANEncoder turnEncoder, moveEncoder;

    Wheel(CANSparkMax turnMotor, CANSparkMax moveMotor) {
        this.turnMotor = turnMotor;
        turnEncoder = new CANEncoder(turnMotor);
        this.moveMotor = moveMotor;
        moveEncoder = new CANEncoder(moveMotor);
    }

    public double turnEncAngle() {
        return (turnEncoder.getPosition() / (double) turnEncoder.getCPR() * 360);
    }
}

