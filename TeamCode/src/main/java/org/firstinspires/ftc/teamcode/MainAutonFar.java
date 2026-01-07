package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.tasks.WaitTask;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@Autonomous(name="AUTON_WITH_PEDRO", group="Linear OpMode")
public class MainAutonFar extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE
    }

    public static double RED_TARGET_X = 135;
    public static double RED_TARGET_Y = 135;
    public static double BLUE_TARGET_X = 14;
    public static double BLUE_TARGET_Y = 135;

    public static double BLUE_STARTING_X = 57;
    public static double BLUE_STARTING_Y = 8.5;
    public static double BLUE_STARTING_HEADING = 0.5 * Math.PI;

    public static double BLUE_PRELOAD_X = 61;;
    public static double BLUE_PRELOAD_Y = 22;
    public static double BLUE_PRELOAD_HEADING = 0.5 * Math.PI;

    public static double BLUE_FIRST_ENDPOINT_X = 41;
    public static double BLUE_FIRST_ENDPOINT_Y = 36;
    public static double BLUE_FIRST_ENDPOINT_HEADING = Math.PI;

    public static Alliance alliance = Alliance.BLUE;

    private Path goToPreload;
    private Path goToFirstEndpoint;

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

        MyRobot.follower = Constants.createFollower(hardwareMap);

        Follower follower = MyRobot.follower;

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
            }

            telemetry.addLine("Alliance: " + alliance.name());

            telemetry.update();
        }


        // If the alliance is BLUE, use the positions as
        // If it is RED, mirror the x coordinate about x = 72 (the field centerline)
        // Also if it is RED, invert the heading
        Pose startingPose;

        robotContext.TURRET.resetEncoder();

        if (alliance == Alliance.BLUE) {
            startingPose = new Pose(
                    BLUE_STARTING_X,
                    BLUE_STARTING_Y,
                    BLUE_STARTING_HEADING
            );
        } else {
            startingPose = new Pose(
                    144 - BLUE_STARTING_X,
                    BLUE_STARTING_Y,
                    Math.PI-BLUE_STARTING_HEADING
            );
        }

        follower.setStartingPose(startingPose);

        double targetX = (alliance == Alliance.BLUE) ? BLUE_TARGET_X : RED_TARGET_X;
        double targetY = (alliance == Alliance.BLUE) ? BLUE_TARGET_Y : RED_TARGET_Y;

        follower.update();

        follower.startTeleopDrive(true);

        buildPaths(startingPose);

        Task mainTask = new SequentialTask(robotContext,
                getGoToPreloadTask(robotContext, follower),
                new WaitTask(robotContext, 1),
                getShootThreeTask(robotContext),
                getGoToFirstEndpointTask(robotContext, follower)
        );

        while (opModeIsActive()){
            follower.update();

            Pose currentPose = follower.getPose();

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = Math.pow(currentPose.getX() - targetX, 2) + Math.pow(currentPose.getY() - targetY, 2);
            robotContext.SHOOTER.setHoodByDistance(d);
            robotContext.SHOOTER.setVelByDistance(d);

            mainTask.step();
        }
    }

    private void buildPaths(Pose startingPose) {
        double preloadX, preloadY, preloadHeading;
        double firstEndpointX, firstEndpointY, firstEndpointHeading;

        if (alliance == Alliance.BLUE) {
            preloadX = BLUE_PRELOAD_X;
            preloadY = BLUE_PRELOAD_Y;
            preloadHeading = BLUE_PRELOAD_HEADING;
            firstEndpointX = BLUE_FIRST_ENDPOINT_X;
            firstEndpointY = BLUE_FIRST_ENDPOINT_Y;
            firstEndpointHeading = BLUE_FIRST_ENDPOINT_HEADING;
        } else {
            preloadX = 144 - BLUE_PRELOAD_X;
            preloadY = BLUE_PRELOAD_Y;
            preloadHeading = Math.PI-BLUE_PRELOAD_HEADING;
            firstEndpointX = 144 - BLUE_FIRST_ENDPOINT_X;
            firstEndpointY = BLUE_FIRST_ENDPOINT_Y;
            firstEndpointHeading =Math.PI -BLUE_FIRST_ENDPOINT_HEADING;
        }

        Pose preloadPose = new Pose(preloadX, preloadY, preloadHeading);
        Pose firstEndpointPose = new Pose(firstEndpointX, firstEndpointY, firstEndpointHeading);

        goToPreload = new Path(new BezierLine(startingPose, preloadPose));
        goToPreload.setLinearHeadingInterpolation(startingPose.getHeading(), preloadPose.getHeading());

        goToFirstEndpoint = new Path(new BezierLine(preloadPose, firstEndpointPose));
        goToFirstEndpoint.setLinearHeadingInterpolation(preloadPose.getHeading(), firstEndpointHeading);
    }

    private Task getGoToPreloadTask(MyRobot robotContext, Follower follower) {
        return new Task(robotContext) {
            @Override
            protected void initialize(RobotContext robotContext) {
                follower.followPath(goToPreload);
            }

            @Override
            protected boolean run(RobotContext robotContext) {
                return follower.isBusy();
            }
        };
    }

    private Task getGoToFirstEndpointTask(MyRobot robotContext, Follower follower) {
        return new Task(robotContext) {
            @Override
            protected void initialize(RobotContext robotContext) {
                follower.followPath(goToFirstEndpoint);
            }

            @Override
            protected boolean run(RobotContext robotContext) {
                return follower.isBusy();
            }
        };
    }

    private Task getShootThreeTask(MyRobot robotContext){
        return new SequentialTask(robotContext,
                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_UP_POS, Transfer.FLAP_TIME_UP_COEFFICIENT),
                robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_DOWN_POS, Transfer.FLAP_TIME_DOWN_COEFFICIENT),
                robotContext.INTAKE.new SetIntakePower(robotContext, 0),
                new WaitTask(robotContext,0.5),
                new ParallelTask(robotContext, false,
                        robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_UP_POS, Transfer.FLAP_TIME_UP_COEFFICIENT),
                        robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_UP_POS, Transfer.FLAP_TIME_UP_COEFFICIENT)
                ),
                new ParallelTask(robotContext, false,
                        robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_DOWN_POS, Transfer.FLAP_TIME_DOWN_COEFFICIENT),
                        robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_DOWN_POS, Transfer.FLAP_TIME_DOWN_COEFFICIENT)
                ),
                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                new WaitTask(robotContext, 1),
                robotContext.INTAKE.new SetIntakePower(robotContext, 0),
                new ParallelTask(robotContext, false,
                        robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_UP_POS, Transfer.FLAP_TIME_UP_COEFFICIENT),
                        robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_UP_POS, Transfer.FLAP_TIME_UP_COEFFICIENT)
                ),
                new ParallelTask(robotContext, false,
                        robotContext.TRANSFER.new MoveLeftTask(robotContext, Transfer.LEFT_DOWN_POS, Transfer.FLAP_TIME_DOWN_COEFFICIENT),
                        robotContext.TRANSFER.new MoveRightTask(robotContext, Transfer.RIGHT_DOWN_POS, Transfer.FLAP_TIME_DOWN_COEFFICIENT)
                ),
                robotContext.INTAKE.new SetIntakePower(robotContext, 0)
        );
    }
}