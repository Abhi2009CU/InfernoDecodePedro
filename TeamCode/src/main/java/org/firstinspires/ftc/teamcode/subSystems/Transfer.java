package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.tasks.QueueTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.util.RobotContext;
import org.firstinspires.ftc.teamcode.MyRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    private final Servo rightServo;
    private final Servo leftServo;
    public static double LEFT_UP_POS = 0.5;
    public static double RIGHT_UP_POS = 0.49;
    public static double LEFT_DOWN_POS = 0.15;
    public static double RIGHT_DOWN_POS = 0.83;

    public Transfer(HardwareMap hardwareMap){
        rightServo = hardwareMap.get(Servo.class, "RightFlap");
        leftServo = hardwareMap.get(Servo.class, "LeftFlap");
    }

    public class MoveLeftTask extends Task {
        private final double pos;
        private final double estimatedTimeTaken;

        public MoveLeftTask(RobotContext robotContext, double pos) {
            super(robotContext);
            this.pos = pos;
            double currentPosition = leftServo.getPosition();
            double TIME_COEFFICIENT = 0.5;
            estimatedTimeTaken = Math.abs(pos - currentPosition) * TIME_COEFFICIENT;
        }

        public void initialize(RobotContext robotContext){
            leftServo.setPosition(pos);
        }

        protected boolean run(RobotContext robotContext){
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }

    }

    public class MoveRightTask extends Task {
        private final double pos;
        private final double estimatedTimeTaken;

        public MoveRightTask(RobotContext robotContext, double pos) {
            super(robotContext);
            this.pos = pos;
            double currentPosition = rightServo.getPosition();
            double TIME_COEFFICIENT = 0.5;
            estimatedTimeTaken = Math.abs(pos - currentPosition) * TIME_COEFFICIENT;
        }

        public void initialize(RobotContext robotContext){
            rightServo.setPosition(pos);

        }

        protected boolean run(RobotContext robotContext){
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }

    }

    public class TransferTask extends QueueTask {
        boolean rightLast = false, leftLast = false;

        public TransferTask(MyRobot robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {}

        @Override
        protected boolean run(RobotContext robotContextWrapper) {
            super.run(robotContextWrapper);

            MyRobot robotContext = (MyRobot) robotContextWrapper;

            if (robotContext.gamepad1.right_trigger > 0 && !rightLast) {
                rightLast = true;
                this.addTask(robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_UP_POS));
                this.addTask(robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_DOWN_POS));
            }else if (robotContext.gamepad1.left_trigger > 0 && !leftLast) {
                leftLast = true;
                this.addTask(robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_UP_POS));
                this.addTask(robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_DOWN_POS));
            }else {
                if (robotContext.gamepad1.right_trigger == 0) {
                    rightLast = false;
                }
                if (robotContext.gamepad1.left_trigger == 0) {
                    leftLast = false;
                }
            }
            boolean running = this.step();

            return running || robotContext.gamepad1.right_trigger > 0 || robotContext.gamepad1.left_trigger > 0;
        }
    }
}
