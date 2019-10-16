/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.UnaryOperator;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

public class Wheel {
    
    CANSparkMax turnMotor, moveMotor;
    CANEncoder turnEncoder, moveEncoder;
    PIDController turnPID, movePID; // <-- sure we need movePID?
    // maybe in auton
    private double targetAngle;
    private double targetSpeed;
    public Vector turnRightVector;

    Wheel(CANSparkMax turnMotor, CANSparkMax moveMotor, Vector turnRightVector) {
        this.turnMotor = turnMotor;
        turnEncoder = new CANEncoder(turnMotor);
        turnPID = new PIDController(1e-5, 1e-8, 1e-2);
        this.moveMotor = moveMotor;
        moveEncoder = new CANEncoder(moveMotor);
        this.turnRightVector = turnRightVector.normalize();
    }

    /**
     * Sets the target velocity of the wheel
     */
    public void setTargetVelocity(Vector velocity) {
        targetAngle = velocity.angle();
        targetSpeed = velocity.magnitude();
        if (targetSpeed < -1 || targetSpeed > 1) {
            targetSpeed = 0; // ask evan
            throw new RuntimeException("Magnitude of velocity vector is too high.");
        }
    }

    /**
     * This function sets the wheel's target angle and speed based on its target vector. It should be called by the robot every tick.
     */
    public void applyVectorMovement() {
        double currentAngle = getTurnAngle();
        double angleDifference = subtractAngles(targetAngle, currentAngle);
        turnPID.input(angleDifference);
        moveMotor.set(targetSpeed); // TODO move speed cap is sqrt(2). should be 1
        turnMotor.set(turnPID.getCorrection());
    }
    
    // https://stackoverflow.com/questions/7570808/how-do-i-calculate-the-difference-of-two-angle-measures
    // public static double distance(double target, double beta) {
    //     int phi = Math.abs(beta - alpha) % 360; // This is either the distance or 360 - distance
    //     int distance = phi > 180 ? 360 - phi : phi;
    //     return distance;
    // }
    // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles

    private double subtractAngles(double target, double current) {
        UnaryOperator<Double> constrainAngleToPositive = a -> {
            a = 360 -(-a % 360);
            a = a % 360;
            return a;
        };
        double absoluteDifference = constrainAngleToPositive.apply(target - current);
        return absoluteDifference > 180 ? absoluteDifference - 360 : absoluteDifference;
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    private double getTurnAngle() {
        return (turnEncoder.getPosition() % turnEncoder.getCPR() / (double) turnEncoder.getCPR() * 360);
    }
}
