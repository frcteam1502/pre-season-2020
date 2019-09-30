/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

public class Wheel {
    
    CANSparkMax turnMotor, moveMotor;
    CANEncoder turnEncoder, moveEncoder;
    PIDController turnPID, movePID;

    Wheel(CANSparkMax turnMotor, CANSparkMax moveMotor) {
        this.turnMotor = turnMotor;
        turnEncoder = new CANEncoder(turnMotor);
        this.moveMotor = moveMotor;
        moveEncoder = new CANEncoder(moveMotor);
    }

    public void setSpeed(double power) {
        this.moveMotor.set(power); // https://stackify.com/oop-concept-for-beginners-what-is-encapsulation/
    }

    public void setTurn(double power) {
        this.turnMotor.set(power);
    }

    public double getTurnAngle() {
        return (turnEncoder.getPosition() / (double) turnEncoder.getCPR() * 360);
    }
}

