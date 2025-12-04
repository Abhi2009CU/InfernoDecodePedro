package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Turret {

    private final Motor turretMotor;
    private final Motor encoderMotor;
    private final PIDController pid;

    public static final double P_GAIN = 0.1;
    public static final double I_GAIN = 0.0;
    public static final double D_GAIN = 0.0;

    public static final double MAX_POWER = 1.0;
    public static final double MIN_POWER = -1.0;

    public static final double TICKS_PER_REV = 537.7;
    public static final double GEAR_RATIO = 215.0/40.0;

    private double newRotation = 0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = new Motor(hardwareMap, "Turret");
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        encoderMotor = new Motor(hardwareMap, "BackRight");
        encoderMotor.resetEncoder();
        pid = new PIDController(P_GAIN, I_GAIN, D_GAIN);
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
    public static double _calculateTurretRotation(double cX, double cY, double cR, double tX, double tY) {
        double a = tX - cX;
        double b = tY - cY;
        double c = Math.sqrt((a * a) + (b * b));
        double A = Math.asin(1 / c * a);
        // double B = Math.asin(1 / c * b);
        double target = cR - A;
        return target;
    }

    public void setRotation(double cX, double cY, double cR, double tX, double tY) {
        double currentRotation = getCurrentRotation() * GEAR_RATIO + cR;
        double targetRotation = _calculateTurretRotation(cX, cY, cR, tX, tY) * GEAR_RATIO * TICKS_PER_REV;
        double error = targetRotation - currentRotation;

        // error is how the turret finds the shortest path and how PID works
        error = (error + TICKS_PER_REV / 2) % TICKS_PER_REV - TICKS_PER_REV / 2;

        double proposedTarget = currentRotation + error;

        double maxLimit = TICKS_PER_REV * GEAR_RATIO / 4;
        double minLimit = -maxLimit;

        if (proposedTarget > maxLimit) {
            proposedTarget = maxLimit;
        } else if (proposedTarget < minLimit) {
            proposedTarget = minLimit;
        }

        newRotation = proposedTarget;
        pid.setSetPoint(newRotation);
    }

    public void updatePID() {
        double currentPosition = getCurrentRotation();
        double power = pid.calculate(currentPosition);
        power = Math.max(Math.min(power, MAX_POWER), MIN_POWER);
        turretMotor.set(power);
    }

    public double getCurrentRotation() {
        return encoderMotor.getCurrentPosition();
    }

    public void stop() {
        encoderMotor.set(0);
    }
}
