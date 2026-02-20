package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.List;

@Configurable
@TeleOp(name="(TELE) Main", group="Main")
public class MainTeleOp extends LinearOpMode {
    public enum Alliance {
        UNSET,
        RED,
        BLUE
    }

    public enum StartingPositionMode {
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
    }

    StateMachine stateMachine;

    public static double RED_TARGET_X = 135;
    public static double RED_TARGET_Y = 140;
    public static double BLUE_TARGET_X = 9;
    public static double BLUE_TARGET_Y = 140;

    public static double BLUE_FAR_STARTING_X = 57;
    public static double BLUE_FAR_STARTING_Y = 8.5;
    public static double BLUE_FAR_STARTING_HEADING = 0.5 * Math.PI;

    public static double BLUE_CLOSE_STARTING_X = 57;
    public static double BLUE_CLOSE_STARTING_Y = 135;
    public static double BLUE_CLOSE_STARTING_HEADING = 1.5 * Math.PI;

    public static Alliance alliance = Alliance.UNSET;
    public static StartingPositionMode startingPositionMode = StartingPositionMode.CARRY_OVER;

    public static double TARGET_X, TARGET_Y;

    @Override
    public void runOpMode() {
        LimelightLocalizer limelightLocalizer = new LimelightLocalizer(hardwareMap.get(Limelight3A.class, "limelight"));
        limelightLocalizer.setPipeline(0);
        limelightLocalizer.start();

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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeIsActive()){
            stateMachine.step();

            follower.update();

            Pose currentPoseLocalizer = follower.getPose();

            Pose currentPoseLL = limelightLocalizer.getPosePedro();

            if (currentPoseLL != null) {
                // If there is less than a 10 percent difference for the x, y, and heading between the two currentPoses, set the follower pose to the limelight pose.
                // This allows us to correct for any drift in the follower without causing large sudden changes in position.
                double xPercentDiff = Math.abs(currentPoseLL.getX() - currentPoseLocalizer.getX()) / Math.max(currentPoseLL.getX(), currentPoseLocalizer.getX());
                double yPercentDiff = Math.abs(currentPoseLL.getY() - currentPoseLocalizer.getY()) / Math.max(currentPoseLL.getY(), currentPoseLocalizer.getY());
                double headingPercentDiff = Math.abs(currentPoseLL.getHeading() - currentPoseLocalizer.getHeading()) / Math.max(Math.abs(currentPoseLL.getHeading()), Math.abs(currentPoseLocalizer.getHeading()));

                // Also allow the driver to manually trigger a pose reset with the circle button, in case the limelight gets a good reading but the follower is significantly off for some reason
                if ((xPercentDiff < 0.1 && yPercentDiff < 0.1 && headingPercentDiff < 0.1) || gamepad2.circle) {
                    follower.setPose(currentPoseLL);
                    currentPoseLocalizer = currentPoseLL;
                }
            }

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPoseLocalizer.getX(), currentPoseLocalizer.getY(), currentPoseLocalizer.getHeading(), TARGET_X, TARGET_Y));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = getDistanceToTarget();
            robotContext.SHOOTER.setHoodByDistance(d, telemetryM);

            if (gamepad2.right_bumper) {
                robotContext.TURRET.incrementAngleOffset(0.01);
            }
            if (gamepad2.left_bumper) {
                robotContext.TURRET.incrementAngleOffset(-0.01);
            }
            if(gamepad2.triangle) {
                robotContext.TURRET.setAngleOffset(0);
            }
            if(gamepad2.dpad_down) {
                robotContext.TURRET.resetEncoder();
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
            if (gamepad2.dpad_right){
                robotContext.SHOOTER.incrementShooterOffset(0.001);
            }
            if (gamepad2.dpad_left){
                robotContext.SHOOTER.incrementShooterOffset(-0.001);
            }
            if (gamepad2.square){
                robotContext.SHOOTER.setShooterOffset(0);
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

            telemetryM.debug("follower pose", currentPoseLocalizer.toString());
            telemetryM.debug("Distance from goal", d);
            telemetryM.addData("Current Velocity", robotContext.SHOOTER.getVelocity());
            telemetryM.addData("Target Velocity", robotContext.SHOOTER.getTargetVelocity());

            double[] positionError = robotContext.TURRET.estimatePositionErrorFromAngleOffset(
                    currentPoseLocalizer.getX(),
                    currentPoseLocalizer.getY(),
                    currentPoseLocalizer.getHeading(),
                    TARGET_X,
                    TARGET_Y
            );

            telemetryM.addData("estimated position error", Math.sqrt(Math.pow(positionError[0], 2) + Math.pow(positionError[1], 2)));

            telemetryM.update();
        }
    }

    public double getDistanceToTarget() {
        return Math.sqrt(Math.pow(MyRobot.follower.getPose().getX() - TARGET_X, 2) + Math.pow(MyRobot.follower.getPose().getY() - TARGET_Y, 2));
    }
}