package org.firstinspires.ftc.teamcode.compOpmode;

import com.bylazar.configurables.annotations.Configurable;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.List;


@Configurable
@Autonomous(name="CloseBoldAuton", group="Main")
public class CloseBoldAuton extends LinearOpMode {

    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance = Alliance.BLUE;
    private MyRobot robotContext;
    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        MyRobot.follower = Constants.createFollower(hardwareMap);
        follower = MyRobot.follower;

        robotContext = new MyRobot(
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

        follower = MyRobot.follower;


//        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = Alliance.RED;
            }

            telemetry.addLine("Alliance: " + alliance.name());
            telemetry.update();
        }

        Pose startingPose = Paths.mirror(new Pose(27.316, 131.650, Math.toRadians(143)));
        follower.setStartingPose(startingPose);


        robotContext.TURRET.resetEncoder();

        Paths paths = new Paths(follower);

        SequentialTask mainTask = buildMainTask(paths);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeIsActive()) {
            mainTask.step();
            follower.update();

            Pose currentPose = follower.getPose();

            Pose target = Paths.mirror(new Pose(9, 140, 0));

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), target.getX(), target.getY()));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            double d = Math.pow(currentPose.getX() - target.getX(), 2) + Math.pow(currentPose.getY() - target.getY(), 2);
            robotContext.SHOOTER.setHoodByDistance(d);
            robotContext.SHOOTER.setVelByDistance(d);
        }
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, GatePath;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(24.363, 130.690)),
                                    mirror(new Pose(53.341, 89.224))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(140)),
                            mirrorHeading(Math.toRadians(188))
                    )
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(53.341, 89.224)),
                                    mirror(new Pose(20.488, 83.907))
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(20.488, 83.907)),
                                    mirror(new Pose(53.140, 89.047))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(-172)),
                            mirrorHeading(Math.toRadians(-150))
                    )
                    .setReversed()
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(53.140, 89.047)),
                                    mirror(new Pose(50.267, 59.570)),
                                    mirror(new Pose(18.233, 59.814))
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            GatePath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(17.070, 83.977)),
                                    mirror(new Pose(23.860, 80.116)),
                                    mirror(new Pose(15.628, 69.512))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(90))
                    )
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(18.233, 59.814)),
                                    mirror(new Pose(41.709, 54.163)),
                                    mirror(new Pose(53.233, 88.977))
                            )
                    ).setConstantHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(53.233, 88.977)),
                                    mirror(new Pose(24.814, 87.163))
                            )
                    ).setConstantHeadingInterpolation(
                            mirrorHeading(Math.toRadians(110))
                    )
                    .build();
        }
        public static Pose mirror(Pose pose) {
            if (alliance == Alliance.RED) {
                return new Pose(144 - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
            }
            return pose;
        }
        private double mirrorHeading(double heading) {
            if (alliance == Alliance.RED) {
                return Math.PI - heading;
            }
            return heading;
        }
    }
    public class goToPath extends Task{
        private final PathChain path;
        public goToPath(PathChain path){
            super(robotContext);
            this.path = path;
        }

        @Override
        public void initialize(RobotContext robotContext){
            follower.followPath(path);
        }

        @Override
        protected boolean run(RobotContext robotContext){
            return follower.isBusy();
        }
    }
    public SequentialTask buildMainTask (Paths paths){
        return new SequentialTask(
                robotContext,

                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                new goToPath(paths.Path1),
                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

                new goToPath(paths.Path2),

                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                new goToPath(paths.Path3),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

                new goToPath(paths.Path4),

                new goToPath(paths.GatePath),

                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                new goToPath(paths.Path5),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

                new goToPath(paths.Path6)
        );
    }
}