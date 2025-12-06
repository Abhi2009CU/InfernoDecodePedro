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

    public Shooter(HardwareMap hardwareMap) {
        //HOOD_SERVO = hardwareMap.get(Servo.class, "hoodServo");
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

    public void updatePID() {
        LEFT_WHEEL.setVeloCoefficients(P, I, D);
        RIGHT_WHEEL.setVeloCoefficients(P, I, D);

        LEFT_WHEEL.set(targetVelocity);
        RIGHT_WHEEL.set(targetVelocity);
    }

    public double getVelocity() {
        return (LEFT_WHEEL.getCorrectedVelocity() / LEFT_WHEEL.getRate() + RIGHT_WHEEL.getCorrectedVelocity() / RIGHT_WHEEL.getRate()) / 2.0;
    }

    public class SetHoodPosTask extends Task {

        private final double DESIRED_POS;
        private final double WAIT_TIME;

        public SetHoodPosTask(RobotContext robotContext, double desiredPos) {
            super(robotContext);
            DESIRED_POS = Math.max(0.0, Math.min(1.0, desiredPos));
            WAIT_TIME = 0.1 * Math.abs(DESIRED_POS - HOOD_SERVO.getPosition());
        }

        @Override
        public void initialize(RobotContext robotContext) {
            HOOD_SERVO.setPosition(DESIRED_POS);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return ELAPSED_TIME.seconds() < WAIT_TIME;
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