package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="(TELE) Main", group="Main")
public class MainTeleOp extends LinearOpMode {
    public enum Alliance {
        UNSET,
        RED,
        BLUE
    }

    public enum StartingPositionMode {
        UNSET,
        CLOSE,
        FAR,
        CARRY_OVER
    }

    public static class UnsetAttributeException extends RuntimeException {
        public UnsetAttributeException(String message) {
            super(message);
        }
        public UnsetAttributeException(String message, Throwable cause) {
            super(message, cause);
        }
    };

    StateMachine stateMachine;

    public static double RED_TARGET_X = 135;
    public static double RED_TARGET_Y = 135;
    public static double BLUE_TARGET_X = 14;
    public static double BLUE_TARGET_Y = 135;

    public static double BLUE_FAR_STARTING_X = 57;
    public static double BLUE_FAR_STARTING_Y = 8.5;
    public static double BLUE_FAR_STARTING_HEADING = 0.5 * Math.PI;

    public static double BLUE_CLOSE_STARTING_X = 57;
    public static double BLUE_CLOSE_STARTING_Y = 135;
    public static double BLUE_CLOSE_STARTING_HEADING = 1.5 * Math.PI;

    public static Alliance alliance = Alliance.UNSET;
    public static StartingPositionMode startingPositionMode = StartingPositionMode.UNSET;

    public static double TARGET_X, TARGET_Y;

    @Override
    public void runOpMode() {
        MyRobot robotContext = new MyRobot(
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                new Intake(hardwareMap),
                new Shooter(hardwareMap),
                new Transfer(hardwareMap),
                new Turret(hardwareMap)
        );
        stateMachine = new StateMachine(new IntakingState(robotContext), robotContext);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
            }

            telemetry.addLine("Alliance: " + alliance.name());

            if (gamepad2.left_trigger > 0.2){
                startingPositionMode = StartingPositionMode.CLOSE;
            } else if (gamepad2.right_trigger > 0.2) {
                startingPositionMode = StartingPositionMode.FAR;
            } else if (gamepad2.cross) {
                startingPositionMode = StartingPositionMode.CARRY_OVER;
            }

            telemetry.addLine("Starting Position: " + startingPositionMode.name());

            telemetry.update();
        }
        if (alliance == Alliance.UNSET) {
            telemetry.clear();
            telemetry.addLine("/!\\ Alliance was unset, did you forget to press a button?");
            telemetry.update();
            throw new UnsetAttributeException("/!\\ Alliance was unset, did you forget to press a button?");
        }
        if (startingPositionMode == StartingPositionMode.UNSET) {
            telemetry.clear();
            telemetry.addLine("/!\\ Starting Position Mode was unset, did you forget to press a button?");
            telemetry.update();
            throw new UnsetAttributeException("/!\\ Starting Position Mode was unset, did you forget to press a button?");
        }

        if (startingPositionMode != StartingPositionMode.CARRY_OVER) {
            MyRobot.follower = Constants.createFollower(hardwareMap);
        }

        if (MyRobot.follower == null) {
            MyRobot.follower = Constants.createFollower(hardwareMap);
        }

        Follower follower = MyRobot.follower;


        // If the alliance is BLUE, use the positions as is
        // If it is RED, mirror the x coordinate about x = 72 (the field centerline)
        // Also if it is RED, reflect the heading about PI/2
        // Carry over means we don't need to set the starting pose here

        Pose startingPose;
        switch (startingPositionMode) {
            case CLOSE:
                startingPose = new Pose(
                        (alliance == Alliance.BLUE) ? BLUE_CLOSE_STARTING_X : 144 - BLUE_CLOSE_STARTING_X,
                        BLUE_CLOSE_STARTING_Y,
                        (alliance == Alliance.BLUE) ? BLUE_CLOSE_STARTING_HEADING : Math.PI - BLUE_CLOSE_STARTING_HEADING
                );
                follower.setStartingPose(startingPose);
                robotContext.TURRET.resetEncoder();
                break;
            case FAR:
                startingPose = new Pose(
                        (alliance == Alliance.BLUE) ? BLUE_FAR_STARTING_X : 144 - BLUE_FAR_STARTING_X,
                        BLUE_FAR_STARTING_Y,
                        (alliance == Alliance.BLUE) ? BLUE_FAR_STARTING_HEADING : Math.PI - BLUE_FAR_STARTING_HEADING
                );
                follower.setStartingPose(startingPose);
                robotContext.TURRET.resetEncoder();
                break;
        }

        TARGET_X = (alliance == Alliance.BLUE) ? BLUE_TARGET_X : RED_TARGET_X;
        TARGET_Y = (alliance == Alliance.BLUE) ? BLUE_TARGET_Y : RED_TARGET_Y;

        follower.update();

        follower.startTeleopDrive(true);

        while (opModeIsActive()){
            stateMachine.step();

            follower.update();

            Pose currentPose = follower.getPose();

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), TARGET_X, TARGET_Y));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = getDistanceToTarget();
            robotContext.SHOOTER.setHoodByDistance(d);

            if (gamepad2.right_bumper) {
                robotContext.TURRET.incrementAngleOffset(0.01);
            }
            if (gamepad2.left_bumper) {
                robotContext.TURRET.incrementAngleOffset(-0.01);
            }

            if (gamepad2.right_trigger > 0.1) {
                robotContext.SHOOTER.incrementHoodOffset(0.01);
            }
            if (gamepad2.left_trigger > 0.1) {
                robotContext.SHOOTER.incrementHoodOffset(-0.01);
            }
            if (gamepad2.cross) {
                robotContext.SHOOTER.setHoodOffset(0);
            }

            if(gamepad2.triangle) {
                robotContext.TURRET.setAngleOffset(0);
            }
            if(gamepad2.dpad_right) {
                robotContext.TURRET.resetEncoder();
            }

            if (gamepad1.triangle) {
                if (alliance == Alliance.BLUE) {
                    follower.setPose(new Pose(27, 132, 2.51327));
                } else {
                    follower.setPose(new Pose(117, 132, 0.628319));
                }
            }

            if (alliance == Alliance.BLUE) {
                follower.setTeleOpDrive(
                        Math.pow(gamepad1.left_stick_y, 3),
                        Math.pow(gamepad1.left_stick_x, 3),
                        Math.pow(-gamepad1.right_stick_x, 3),
                        false // field Centric
                );
            } else {
                follower.setTeleOpDrive(
                        Math.pow(-gamepad1.left_stick_y, 3),
                        Math.pow(-gamepad1.left_stick_x, 3),
                        Math.pow(-gamepad1.right_stick_x, 3),
                        false // field Centric
                );
            }

            telemetryM.debug("x pos", currentPose.getX());
            telemetryM.debug("y pos", currentPose.getY());
            telemetryM.debug("heading pos", currentPose.getHeading());
            telemetryM.debug("Distance from goal", d);
            telemetryM.addData("Current Velocity", robotContext.SHOOTER.getVelocity());
            telemetryM.addData("Target Velocity", robotContext.SHOOTER.getTargetVelocity());
            telemetryM.update();
        }
    }

    public double getDistanceToTarget() {
        return Math.sqrt(Math.pow(MyRobot.follower.getPose().getX() - TARGET_X, 2) + Math.pow(MyRobot.follower.getPose().getY() - TARGET_Y, 2));
    }
}