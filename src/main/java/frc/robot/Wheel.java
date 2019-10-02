/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class Wheel {
    
    CANSparkMax turnMotor, moveMotor;
    CANEncoder turnEncoder, moveEncoder;
    PIDController turnPID, movePID; // <-- sure we need movePID?
    // maybe in auton
    Vector2d targetVector;

    Wheel(CANSparkMax turnMotor, CANSparkMax moveMotor) {
        this.turnMotor = turnMotor;
        turnEncoder = new CANEncoder(turnMotor);
        this.moveMotor = moveMotor;
        moveEncoder = new CANEncoder(moveMotor);
    }

    /**
     * Sets the target velocity of the wheel
     */
    public void setTargetVelocity(double x, double y) {
        targetVector.x = x;
        targetVector.y = y;
    }

    /**
     * This function sets the wheel's target angle and speed based on its target vector. It should be called by the robot every tick.
     */
    public void applyVectorMovement() {
        double targetAngleAbsolute = Math.atan2(targetVector.y, targetVector.x);
        double currentAngleAbsolute;
        double targetSpeed = targetVector.magnitude();
        moveMotor.set(targetSpeed);
        
    }

    private void setTargetAngle(double angle) {
        if (getTurnAngle() > angle) turnMotor.set(-1);
        else if (getTurnAngle() < angle) turnMotor.set(1);
    }

    private double getTurnAngle() {
        return (turnEncoder.getPosition() / (double) turnEncoder.getCPR() * 360);
    }
}
