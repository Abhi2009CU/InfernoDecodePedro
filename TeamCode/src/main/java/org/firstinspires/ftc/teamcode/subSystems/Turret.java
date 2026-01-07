package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret {

    private final Motor turretMotor;
    private final Motor encoderRefMotor;
    private final PIDController PID;

    public static double P = 0.95;
    public static double I = 0.03;
    public static double D = 0.02;

    public static double MAX_POWER = 1.0;
    public static double MIN_POWER = -1.0;
    public static double MAX_ANGLE = 5.0/6 * Math.PI;
    public static double MIN_ANGLE = -5.0/6 * Math.PI;

    public static final double TICKS_PER_REV = 384.5;
    public static final double GEAR_RATIO = 215.0/40.0;

    private double angleOffset = 0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = new Motor(hardwareMap, "Turret");
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        encoderRefMotor = new Motor(hardwareMap, "BackRight");
        PID = new PIDController(P, I, D);
    }

    /**
     * <p>Calculate the angle needed for the robot's turret to aim to a specific position on the field.<br />Note that <span style="font-style:italic">either degrees or radians can be used</span>, but you have to be consistent.<br /><br />&quot;I &lt;3 trigonometry!&quot;</p>
     * @param cX The robot's current X position
     * @param cY The robot's current Y position
     * @param cR The robot's current rotation
     * @param tX The target's X position
     * @param tY The target's Y position
     * @return Suggested angle that the turret should face
     */
    public static double calculateGoalRotation(double cX, double cY, double cR, double tX, double tY) {
        double theta = Math.atan2(tY - cY, tX - cX);
        return cR - theta;
    }

    public void setRotation(double theta) {
        double normalizedTheta = ((theta % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
        if (normalizedTheta > Math.PI) normalizedTheta -= 2 * Math.PI;
        double clampedTheta = Math.max(Math.min(normalizedTheta, MAX_ANGLE), MIN_ANGLE);
        PID.setSetPoint(clampedTheta + angleOffset);
    }

    public void updatePID() {
        PID.setPID(P, I, D);
        double currentPosition = getCurrentRotation();
        double power = -PID.calculate(currentPosition);
        power = Math.max(Math.min(power, MAX_POWER), MIN_POWER);
        turretMotor.set(power);
    }

    public void incrementAngleOffset(double i) {
        angleOffset += i;
    }

    public void setAngleOffset (double o) {
        angleOffset = o;
    }

    public double getCurrentRotation() {
        return -encoderRefMotor.getCurrentPosition() / TICKS_PER_REV / GEAR_RATIO * 2 * Math.PI;
    }

    public void resetEncoder() {
        encoderRefMotor.resetEncoder();
    }
}
