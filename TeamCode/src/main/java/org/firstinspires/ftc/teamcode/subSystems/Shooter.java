package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.jumpypants.murphy.util.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MyRobot;

@Configurable
public class Shooter {
    public static double P = 0.3;
    public static double I = 0.2;
    public static double D = 0.0;

    private Servo HOOD_SERVO;
    private final Motor LEFT_WHEEL;
    private final Motor RIGHT_WHEEL;

    private double targetVelocity = 0.0;

    public static double MAX_VELOCITY = 1;
    public static double IDLE_VELOCITY = 0.5;
    public static double STOP_VELOCITY = 0;

    public Shooter(HardwareMap hardwareMap) {
        HOOD_SERVO = hardwareMap.get(Servo.class, "Hood");

        LEFT_WHEEL = new Motor(hardwareMap, "LeftShooter");
        RIGHT_WHEEL = new Motor(hardwareMap, "RightShooter");

        LEFT_WHEEL.setRunMode(Motor.RunMode.VelocityControl);
        RIGHT_WHEEL.setRunMode(Motor.RunMode.VelocityControl);

        LEFT_WHEEL.setVeloCoefficients(P, I, D);
        RIGHT_WHEEL.setVeloCoefficients(P, I, D);

        RIGHT_WHEEL.setInverted(true);
    }

    public void setVel(double vel) {
        targetVelocity = vel;
    }

    private static double calcHoodPos (double d) {
        return d * 0.01 + 0.1;
    }

    public void setHoodByDistance(double distance) {
        setHoodPosition(calcHoodPos(distance));
    }

    public void setHoodPosition(double position) {
        double clampedPosition = Math.max(0.0, Math.min(1.0, position));
        HOOD_SERVO.setPosition(clampedPosition);
    }

    public void updatePID() {
        LEFT_WHEEL.setVeloCoefficients(P, I, D);
        RIGHT_WHEEL.setVeloCoefficients(P, I, D);

        LEFT_WHEEL.set(targetVelocity);
        RIGHT_WHEEL.set(targetVelocity);
    }

    public double getVelocity() {
        return (LEFT_WHEEL.getCorrectedVelocity() / LEFT_WHEEL.getRate() + RIGHT_WHEEL.getCorrectedVelocity() / RIGHT_WHEEL.getRate()) / 2.0;
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
