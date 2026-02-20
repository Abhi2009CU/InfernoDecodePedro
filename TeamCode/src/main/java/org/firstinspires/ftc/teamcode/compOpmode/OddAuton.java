package org.firstinspires.ftc.teamcode.compOpmode;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.tasks.WaitTask;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.ArrayList;
import java.util.List;

@Configurable
@Autonomous(name="Odd Auton Far", group="Main")
public class OddAuton extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE
    }
    public static Alliance alliance = Alliance.BLUE;

    private final Pose START_POSE = new Pose(57, 8.5, 0.5 * Math.PI);

    private final List<Path> paths = new ArrayList<>();

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

        robotContext.TRANSFER.resetLeft();
        robotContext.TRANSFER.resetRight();

        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
            }

            telemetry.addLine("Alliance: " + alliance.name());

            telemetry.update();
        }

        robotContext.TURRET.resetEncoder();

        Pose startingPose = mirrorForAlliance(START_POSE.getX(), START_POSE.getY(), START_POSE.getHeading());
        follower.setStartingPose(startingPose);

        double targetX = (alliance == Alliance.BLUE) ? 9 : 135;
        double targetY = 140;

        follower.update();
        follower.startTeleopDrive(true);

        buildPaths();

        Task mainTask = getMainTask(robotContext, follower, paths);

        while (opModeIsActive()){
            follower.update();

            Pose currentPose = follower.getPose();

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation(currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = Math.sqrt(Math.pow(currentPose.getX() - targetX, 2) + Math.pow(currentPose.getY() - targetY, 2));
            robotContext.SHOOTER.setHoodByDistance(d);
            robotContext.SHOOTER.setVelByDistance(d);

            mainTask.step();
        }
    }

    @NonNull
    private Task getMainTask(MyRobot robotContext, Follower follower, List<Path> paths) {
        return new SequentialTask(robotContext,
                new FollowPathTask(robotContext, follower, paths.get(0)),
                new WaitTask(robotContext, 1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),

                new FollowPathTask(robotContext, follower, paths.get(1))
        );
    }

    private void buildPaths() {
        addLinePath(paths, 61, 22, 0.5 * Math.PI);
        addLinePath(paths,
                36.326, 11, 0.5 * Math.PI
        );
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

    private Pose getLastPose(List<Path> pathList) {
        return pathList.isEmpty() ?
                mirrorForAlliance(START_POSE.getX(), START_POSE.getY(), START_POSE.getHeading()) :
                pathList.get(pathList.size() - 1).getLastControlPoint();
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