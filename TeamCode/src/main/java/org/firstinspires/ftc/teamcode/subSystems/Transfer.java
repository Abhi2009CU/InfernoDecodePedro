package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.QueueTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.util.RobotContext;
import org.firstinspires.ftc.teamcode.MyRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Transfer {
    public static double FLAP_TIME_UP_COEFFICIENT = 2;//2;
    public static double FLAP_TIME_DOWN_COEFFICIENT = 2;//2;

    private final Servo rightFlap;
    private final Servo leftFlap;
    public static double LEFT_UP_POS = 0.48;
    public static double RIGHT_UP_POS = 0.49;
    public static double LEFT_DOWN_POS = 0.15;
    public static double RIGHT_DOWN_POS = 0.82;

    public Transfer(HardwareMap hardwareMap){
        rightFlap = hardwareMap.get(Servo.class, "RightFlap");
        leftFlap = hardwareMap.get(Servo.class, "LeftFlap");
    }

    public class MoveLeftTask extends Task {
        private final double pos;
        private final double estimatedTimeTaken;

        public MoveLeftTask(RobotContext robotContext, double pos, double coeff) {
            super(robotContext);
            this.pos = pos;
            double currentPosition = leftFlap.getPosition();
            estimatedTimeTaken = coeff;
        }

        public void initialize(RobotContext robotContext){
            leftFlap.setPosition(pos);
        }

        protected boolean run(RobotContext robotContext){
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }

    }

    public class MoveRightTask extends Task {
        private final double pos;
        private final double estimatedTimeTaken;

        public MoveRightTask(RobotContext robotContext, double pos, double coeff) {
            super(robotContext);
            this.pos = pos;
            double currentPosition = rightFlap.getPosition();
            estimatedTimeTaken = coeff;
        }

        public void initialize(RobotContext robotContext){
            rightFlap.setPosition(pos);

        }

        protected boolean run(RobotContext robotContext){
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }

    }

    public class TransferTask extends QueueTask {

        public TransferTask(MyRobot robotContext) {
            super(robotContext);
        }

        @Override
        protected boolean run(RobotContext robotContextWrapper) {
            super.run(robotContextWrapper);

            MyRobot robot = (MyRobot) robotContextWrapper;

            if (robot.GAMEPAD1.rightBumperWasPressed()) {
                this.addTask(robot.TRANSFER.new MoveRightTask(robot, Transfer.RIGHT_UP_POS, FLAP_TIME_UP_COEFFICIENT));
                this.addTask(robot.TRANSFER.new MoveRightTask(robot, Transfer.RIGHT_DOWN_POS, FLAP_TIME_DOWN_COEFFICIENT));
            }

            if (robot.GAMEPAD1.leftBumperWasPressed()) {
                this.addTask(robot.TRANSFER.new MoveLeftTask(robot, Transfer.LEFT_UP_POS, FLAP_TIME_UP_COEFFICIENT));
                this.addTask(robot.TRANSFER.new MoveLeftTask(robot, Transfer.LEFT_DOWN_POS, FLAP_TIME_DOWN_COEFFICIENT));
            }

            if (robot.GAMEPAD1.aWasPressed()) {
                this.addTask(new ParallelTask(robot, false,
                        robot.TRANSFER.new MoveRightTask(robot, Transfer.RIGHT_UP_POS, FLAP_TIME_UP_COEFFICIENT),
                        robot.TRANSFER.new MoveLeftTask(robot, Transfer.LEFT_UP_POS, FLAP_TIME_UP_COEFFICIENT)));
                this.addTask(new ParallelTask(robot, false,
                        robot.TRANSFER.new MoveRightTask(robot, Transfer.RIGHT_DOWN_POS, FLAP_TIME_DOWN_COEFFICIENT),
                        robot.TRANSFER.new MoveLeftTask(robot, Transfer.LEFT_DOWN_POS, FLAP_TIME_DOWN_COEFFICIENT)));
            }

            return super.run(robotContextWrapper);
        }
    }

    public class ManualControlTask extends Task {

        public ManualControlTask(MyRobot robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {}

        @Override
        protected boolean run(RobotContext robotContextWrapper) {
            MyRobot robot = (MyRobot) robotContextWrapper;

            if (robot.GAMEPAD1.right_bumper || robot.GAMEPAD1.a) {
                rightFlap.setPosition(Transfer.RIGHT_UP_POS);
            } else {
                rightFlap.setPosition(Transfer.RIGHT_DOWN_POS);
            }

            if (robot.GAMEPAD1.left_bumper || robot.GAMEPAD1.a) {
                leftFlap.setPosition(Transfer.LEFT_UP_POS);
            } else {
                leftFlap.setPosition(Transfer.LEFT_DOWN_POS);
            }

            if (robot.GAMEPAD2.dpadUpWasPressed()){
                robot.GAMEPAD1.rumble(200);
                robot.GAMEPAD2.rumble(200);
                return false;
            }
            return true;
        }
    }
}