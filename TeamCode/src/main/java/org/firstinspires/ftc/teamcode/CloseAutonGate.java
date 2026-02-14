package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.List;

@Configurable
@Autonomous(name="MainAutonClose - Normal", group="Main")
public class CloseAutonGate extends LinearOpMode {

    private MyRobot robotContext;
    Follower follower;
    double d;

    public enum Alliance {
        RED,
        BLUE
    }
    public static Alliance alliance = Alliance.BLUE;

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

        Pose startingPose = new Pose(25, 129, Math.toRadians(143));
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

            Pose target = new Pose(14, 135, Math.toRadians(0)); //basket

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), target.getX(), target.getY()));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            d = Math.pow(currentPose.getX() - target.getX(), 2) + Math.pow(currentPose.getY() - target.getY(), 2);
            d = Math.sqrt(d);
            telemetryM.addLine("Distance to target: " + d);
            telemetry.addLine(("Distance to target: " + d));
            telemetryM.addLine("Target Velocity: " + robotContext.SHOOTER.getTargetVelocity());
            telemetry.addLine("Current Heading" + Math.toDegrees(currentPose.getHeading()));
            telemetryM.update();
            robotContext.SHOOTER.setHoodByDistance(d);
            robotContext.SHOOTER.setVelByDistance(d);
        }
    }
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(27.116, 132.326)),
                                    mirror(new Pose(54.535, 89.372))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(143)),
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(54.535, 89.372)),
                                    mirror(new Pose(21.000, 83.791))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(190))
                    )
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(21.000, 83.791)),
                                    mirror(new Pose(54.465, 89.256))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(190)),
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(54.465, 89.256)),
                                    mirror(new Pose(62.895, 59.465)),
                                    mirror(new Pose(20.721, 54.442))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(20.721, 59.442)),
                                    mirror(new Pose(54.093, 89.233))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(54.093, 89.233)),
                                    mirror(new Pose(24.651, 88.628))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(90))
                    )
                    .build();
        }
        public static Pose mirror(Pose pose) {
            if (alliance == Alliance.RED) {
                return new Pose(144 - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
            }
            return pose;
        }
        private double mirrorHeading(double headingRad) {
            if (alliance == Alliance.RED) {
                return Math.PI - headingRad;
            }
            return headingRad;
        }
    }
    public class goToPath extends Task {
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

                new goToPath(paths.Path1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),

                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                new goToPath(paths.Path2),

                new goToPath(paths.Path3),
                robotContext.INTAKE.new SetIntakePower(robotContext, 0),
                robotContext.TRANSFER.new SendThreeTask(robotContext),

                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                new goToPath(paths.Path4),

                new goToPath(paths.Path5),
                robotContext.INTAKE.new SetIntakePower(robotContext, 0),
                robotContext.TRANSFER.new SendThreeTask(robotContext),

                new goToPath(paths.Path6)
        );
    }



}
