package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.ArrayList;
import java.util.List;

@Configurable
@Autonomous(name="Main Auton Far", group="Main")
public class MainAutonFar extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE
    }
    public static Alliance alliance = Alliance.BLUE;
    public static boolean openGate = false;

    private final Pose START_POSE = new Pose(57, 8.5, 0.5 * Math.PI);

    private final List<Path> pathsClosed = new ArrayList<>();
    private final List<Path> pathsOpen = new ArrayList<>();

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

            if (gamepad2.a) {
                openGate = false;
            } else if (gamepad2.b) {
                openGate = true;
            }

            telemetry.addLine("Alliance: " + alliance.name());
            telemetry.addLine("Open Gate: " + (openGate ? "YES" : "NO"));

            telemetry.update();
        }

        robotContext.TURRET.resetEncoder();

        Pose startingPose = mirrorForAlliance(START_POSE.getX(), START_POSE.getY(), START_POSE.getHeading());
        follower.setStartingPose(startingPose);

        double targetX = (alliance == Alliance.BLUE) ? 14 : 135;
        double targetY = 135;

        follower.update();
        follower.startTeleopDrive(true);

        buildPathsClosed();
        buildPathsOpen();

        List<Path> paths = openGate ? pathsOpen : pathsClosed;
        Task mainTask = getMainTask(robotContext, follower, paths);

        while (opModeIsActive()){
            follower.update();

            Pose currentPose = follower.getPose();

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation(currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = Math.pow(currentPose.getX() - targetX, 2) + Math.pow(currentPose.getY() - targetY, 2);
            robotContext.SHOOTER.setHoodByDistance(d);
            robotContext.SHOOTER.setVelByDistance(d);

            mainTask.step();
        }
    }

    @NonNull
    private Task getMainTask(MyRobot robotContext, Follower follower, List<Path> paths) {
        if (openGate) {
            return getOpenGateTask(robotContext, follower, paths);
        } else {
            return getClosedGateTask(robotContext, follower, paths);
        }
    }

    @NonNull
    private Task getClosedGateTask(MyRobot robotContext, Follower follower, List<Path> paths) {
        return new SequentialTask(robotContext,
                    new FollowPathTask(robotContext, follower, paths.get(0)),
                    new WaitTask(robotContext, 1),
                    robotContext.TRANSFER.new SendThreeTask(robotContext),

                    robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                    new FollowPathTask(robotContext, follower, paths.get(1)),
                    new FollowPathTask(robotContext, follower, paths.get(2)),
                    robotContext.TRANSFER.new SendThreeTask(robotContext),

                    robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                    new FollowPathTask(robotContext, follower, paths.get(3)),
                    new WaitTask(robotContext, 0.5),
                    new FollowPathTask(robotContext, follower, paths.get(4)),
                    robotContext.TRANSFER.new SendThreeTask(robotContext),

                    robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                    new FollowPathTask(robotContext, follower, paths.get(5)),
                    new WaitTask(robotContext, 0.5),
                    new FollowPathTask(robotContext, follower, paths.get(6)),
                    robotContext.TRANSFER.new SendThreeTask(robotContext),

                    new FollowPathTask(robotContext, follower, paths.get(7))
            );
    }

    @NonNull
    private Task getOpenGateTask(MyRobot robotContext, Follower follower, List<Path> paths) {
        return new SequentialTask(robotContext,
                    new FollowPathTask(robotContext, follower, paths.get(0)),
                    new WaitTask(robotContext, 1),
                    robotContext.TRANSFER.new SendThreeTask(robotContext),

                    robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                    new FollowPathTask(robotContext, follower, paths.get(1)),
                    new FollowPathTask(robotContext, follower, paths.get(2)),
                    robotContext.TRANSFER.new SendThreeTask(robotContext),

                    robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                    new FollowPathTask(robotContext, follower, paths.get(3)),
                    new FollowPathTask(robotContext, follower, paths.get(4)),
                    new WaitTask(robotContext, 0.3),
                    new FollowPathTask(robotContext, follower, paths.get(5)),
                    robotContext.TRANSFER.new SendThreeTask(robotContext),

                    robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                    new FollowPathTask(robotContext, follower, paths.get(6)),
                    new WaitTask(robotContext, 0.5),
                    new FollowPathTask(robotContext, follower, paths.get(7)),
                    robotContext.TRANSFER.new SendThreeTask(robotContext),

                    new FollowPathTask(robotContext, follower, paths.get(8))
        );
    }

    private void buildPathsClosed() {
        addLinePath(pathsClosed, 61, 22, 0.5 * Math.PI);
        addBezierPathWithTangent(pathsClosed, 15, 36,
                62, 36,
                37, 36
        );
        addLinePathConstant(pathsClosed, 61, 22, Math.PI);
        addBezierPath(pathsClosed, 23, 10.5, (double) 10/9 * Math.PI,
                58, 9.5
        );
        addLinePath(pathsClosed, 61, 22, Math.PI);
        addBezierPath(pathsClosed, 23, 10.5, (double) 10/9 * Math.PI,
                58, 9.5
        );
        addLinePath(pathsClosed, 61, 22, Math.PI);
        addLinePath(pathsClosed, 60, 30, Math.PI);
    }

    private void buildPathsOpen() {
        addLinePath(pathsOpen, 61, 22, 0.5 * Math.PI);
        addBezierPathWithTangent(pathsOpen, 15, 36,
                62, 36,
                37, 36
        );
        addLinePathConstant(pathsOpen, 61, 22, Math.PI);
        addBezierPathWithTangent(pathsOpen, 15, 60,
                62, 68,
                37, 59
        );
        addBezierPath(pathsOpen, 15.5, 70, 0.5 * Math.PI,
                31, 66.5
        );
        addLinePathConstant(pathsOpen, 61, 22, Math.PI);
        addBezierPath(pathsOpen, 23, 10.5, (double) 10/9 * Math.PI,
                58, 9.5
        );
        addLinePath(pathsOpen, 61, 22, Math.PI);
        addLinePath(pathsOpen, 60, 30, Math.PI);
    }

    private Pose mirrorForAlliance(double x, double y, double heading) {
        if (alliance == Alliance.BLUE) {
            return new Pose(x, y, heading);
        } else {
            return new Pose(144 - x, y, Math.PI - heading);
        }
    }

    private void addLinePath(List<Path> pathList, double x, double y, double heading) {
        Pose start = getLastPose(pathList);
        Pose end = mirrorForAlliance(x, y, heading);

        Path path = new Path(new BezierLine(start, end));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        pathList.add(path);
    }

    private void addLinePathConstant(List<Path> pathList, double x, double y, double heading) {
        Pose start = getLastPose(pathList);
        Pose end = mirrorForAlliance(x, y, heading);

        Path path = new Path(new BezierLine(start, end));
        path.setConstantHeadingInterpolation(end.getHeading());
        pathList.add(path);
    }

    private void addBezierPath(List<Path> pathList, double x, double y, double heading, double... controlPoints) {
        Pose start = getLastPose(pathList);
        Pose end = mirrorForAlliance(x, y, heading);
        Pose[] controlPoses = buildControlPoses(start, end, controlPoints);

        Path path = new Path(new BezierCurve(controlPoses));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
        pathList.add(path);
    }

    private void addBezierPathWithTangent(List<Path> pathList, double x, double y, double... controlPoints) {
        Pose start = getLastPose(pathList);
        Pose end = mirrorForAlliance(x, y, 0);
        Pose[] controlPoses = buildControlPoses(start, end, controlPoints);

        Path path = new Path(new BezierCurve(controlPoses));
        path.setTangentHeadingInterpolation();
        pathList.add(path);
    }

    private Pose getLastPose(List<Path> pathList) {
        return pathList.isEmpty() ?
            mirrorForAlliance(START_POSE.getX(), START_POSE.getY(), START_POSE.getHeading()) :
            pathList.get(pathList.size() - 1).getLastControlPoint();
    }

    private Pose[] buildControlPoses(Pose start, Pose end, double... controlPoints) {
        Pose[] controlPoses = new Pose[controlPoints.length / 2 + 2];
        controlPoses[0] = start;

        for (int i = 0; i < controlPoints.length; i += 2) {
            controlPoses[i / 2 + 1] = mirrorForAlliance(controlPoints[i], controlPoints[i + 1], 0);
        }

        controlPoses[controlPoses.length - 1] = end;
        return controlPoses;
    }

    private static class FollowPathTask extends Task {
        private final Follower follower;
        private final Path path;

        public FollowPathTask(RobotContext robotContext, Follower follower, Path path) {
            super(robotContext);
            this.follower = follower;
            this.path = path;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            follower.followPath(path);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return follower.isBusy();
        }
    }
}