package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.util.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.InterpLUT;
import org.firstinspires.ftc.teamcode.MyRobot;

@Configurable
public class Shooter {
    private final InterpLUT HOOD_POSITION_LUT = new InterpLUT();
    private final InterpLUT VELOCITY_LUT = new InterpLUT();

    public static double FAR_SHOOT_VEL = 0.8;
    public static double MID_SHOOT_VEL = 0.7;
    public static double CLOSE_SHOOT_VEL = 0.68;

    public static double SUPER_CLOSE_SHOOT_VEL = 0.6;

    public static double IDLE_VEL = 0.3;

    public static double FAR_SHOOT_HOOD = 0.15;
    public static double MID_SHOOT_HOOD = 0.15;
    public static double CLOSE_SHOOT_HOOD = 0.17;
    public static double SUPER_CLOSE_SHOOT_HOOD = 0.4;

    public static double P = 0.15;
    public static double I = 0.0;
    public static double D = 0.0;

    public static double V = 1.255;
    public static double S = 0;

    private Servo HOOD_SERVO;
    public final double HOOD_MAX_POS = 0.45;
    public final double HOOD_MIN_POS = 0;
    private final Motor LEFT_WHEEL;
    private final Motor RIGHT_WHEEL;

    private double targetVelocity = 0.0;

    private double hoodOffset = 0;

    public Shooter(HardwareMap hardwareMap) {
        HOOD_SERVO = hardwareMap.get(Servo.class, "Hood");

        LEFT_WHEEL = new Motor(hardwareMap, "LeftShooter");
        RIGHT_WHEEL = new Motor(hardwareMap, "RightShooter");

        LEFT_WHEEL.setRunMode(Motor.RunMode.VelocityControl);
        RIGHT_WHEEL.setRunMode(Motor.RunMode.VelocityControl);

        LEFT_WHEEL.setVeloCoefficients(P, I, D);
        RIGHT_WHEEL.setVeloCoefficients(P, I, D);
        LEFT_WHEEL.setFeedforwardCoefficients(S, V);
        RIGHT_WHEEL.setFeedforwardCoefficients(S, V);

        RIGHT_WHEEL.setInverted(true);

        HOOD_POSITION_LUT.add(50, SUPER_CLOSE_SHOOT_HOOD);
        HOOD_POSITION_LUT.add(60, CLOSE_SHOOT_HOOD);
        HOOD_POSITION_LUT.add(88, MID_SHOOT_HOOD);
        HOOD_POSITION_LUT.add(111, FAR_SHOOT_HOOD);

        VELOCITY_LUT.add(50, SUPER_CLOSE_SHOOT_VEL);
        VELOCITY_LUT.add(60, CLOSE_SHOOT_VEL);
        VELOCITY_LUT.add(88, MID_SHOOT_VEL);
        VELOCITY_LUT.add(111, FAR_SHOOT_VEL);

    }

    public void setVel(double vel) {
        targetVelocity = vel;
    }

    private double calcHoodPos (double d) {
        return HOOD_POSITION_LUT.get(d);
    }

    private double calcVelocity (double d) {
        return VELOCITY_LUT.get(d);
    }

    public void setHoodByDistance(double distance) {
        setHoodPosition(calcHoodPos(distance));
    }

    public void setHoodByDistance(double distance, TelemetryManager telemetry) {
        setHoodPosition(calcHoodPos(distance), telemetry);
    }

    public void setVelByDistance(double distance) {
        setVel(calcVelocity(distance));
    }

    public void setHoodPosition(double position) {
        position += hoodOffset;
        double clampedPosition = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
        HOOD_SERVO.setPosition(clampedPosition);
    }
    public void setHoodPosition(double position, TelemetryManager telemetry) {
        position += hoodOffset;
        double clampedPosition = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
        telemetry.debug("target hood pos", clampedPosition);
        HOOD_SERVO.setPosition(clampedPosition);
    }

    public void incrementHoodOffset(double i) {
        hoodOffset += i;
    }

    public void setHoodOffset(double o) {
        hoodOffset = o;
    }

    public void updatePID() {
        LEFT_WHEEL.setVeloCoefficients(P, I, D);
        RIGHT_WHEEL.setVeloCoefficients(P, I, D);

        LEFT_WHEEL.setFeedforwardCoefficients(S, V);
        RIGHT_WHEEL.setFeedforwardCoefficients(S, V);

        LEFT_WHEEL.set(targetVelocity);
        RIGHT_WHEEL.set(targetVelocity);
    }

    public double getVelocity() {
        double maxTicksPerSec = (6000.0 * 28.0) / 60.0;
        double currentVelocityTicksPerSecLEFT = LEFT_WHEEL.getCorrectedVelocity();
        double currentVelocityTicksPerSecRIGHT = RIGHT_WHEEL.getCorrectedVelocity();

        double normalizedVelocity = (currentVelocityTicksPerSecLEFT / maxTicksPerSec) + (currentVelocityTicksPerSecRIGHT / maxTicksPerSec);
        normalizedVelocity /= 2;

        return normalizedVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public class setVelocity extends Task{

        public setVelocity(RobotContext robotContext, double vel){
            super(robotContext);
            setVel(vel);
        }

        @Override
        protected void initialize(RobotContext robotContext) {
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return false;
        }
    }

    public class RunOuttakeTask extends Task {

        private final double POWER;

        public RunOuttakeTask(MyRobot robotContext, double power) {
            super(robotContext);
            POWER = power;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            LEFT_WHEEL.set(POWER);
            RIGHT_WHEEL.set(POWER);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return false;
        }
    }
}
