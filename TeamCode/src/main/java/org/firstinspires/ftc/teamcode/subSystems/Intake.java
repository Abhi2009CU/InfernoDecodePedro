package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.util.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MyRobot;
public class Intake {
    private final Motor INTAKE_MOTOR;
    public final int MAXPOWER = 1, STOPPOWER = 0;

    public Intake(HardwareMap hardwareMap) {
        INTAKE_MOTOR = new Motor(hardwareMap, "INTAKE_MOTOR");
        INTAKE_MOTOR.setRunMode(Motor.RunMode.RawPower);
        INTAKE_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }

    public class SetIntakePower extends Task {
        private final double POWER;

        public SetIntakePower(MyRobot robotContext, double power) {
            super(robotContext);
            this.POWER = power;
        }


        @Override
        protected void initialize(RobotContext robotContext) {
            INTAKE_MOTOR.set(POWER);
        }

        @Override
        protected boolean run(RobotContext robotContext) { //to be cont.
            return false;
        }
    }



    public class ManualRunIntakeMotor extends Task {
        public ManualRunIntakeMotor (RobotContext robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {}

        @Override
        protected boolean run(RobotContext robotContextWrapper) {
            MyRobot robotContext = (MyRobot) robotContextWrapper;

            double intakePower = Math.max(-1.0, Math.min(1.0, robotContext.gamepad1.right_trigger - robotContext.gamepad1.left_trigger));
            INTAKE_MOTOR.set(intakePower);

            // For when we want to stop intaking and start transfer
            return !robotContext.gamepad1.dpadUpWasPressed();
        }
    }
}