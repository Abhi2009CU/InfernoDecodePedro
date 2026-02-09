package org.firstinspires.ftc.teamcode;

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
@Autonomous(name="MainAutonClose-Gate", group="Main")
public class MainAutonClose_Gate extends LinearOpMode {

    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance = Alliance.BLUE;
    private MyRobot robotContext;
    Follower follower;

    double d;

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

        Pose startingPose = Paths.mirror(new Pose(25, 129, Math.toRadians(143)));
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

            Pose target = Paths.mirror(new Pose(10, 140, Math.toRadians(0))); //basket

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), target.getX(), target.getY()));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            d = Math.pow(currentPose.getX() - target.getX(), 2) + Math.pow(currentPose.getY() - target.getY(), 2);
            d = Math.sqrt(d);
            telemetryM.addLine("Distance to target: " + d);
            telemetry.addLine(("Distance to target: " + d));
            telemetryM.addLine("Target Velocity: " + robotContext.SHOOTER.getTargetVelocity());
            telemetry.addLine("Current Heading" + Math.toDegrees(currentPose.getHeading()));
            //telemetryM.update();
            robotContext.SHOOTER.setHoodByDistance(d);
            robotContext.SHOOTER.setVelByDistance(d);
        }
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(25.618, 129.853)),
                                    mirror(new Pose(50.099, 93.409))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(143)),
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(50.099, 93.409)),
                                    mirror(new Pose(42.110, 83.527)),
                                    mirror(new Pose(24.021, 83.825))
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
                    /*.setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();
                    */

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(24.021, 83.825)),
                                    mirror(new Pose(38.953, 79.458)),
                                    mirror(new Pose(31.009, 91.138)),
                                    mirror(new Pose(50.182, 93.474))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(270))
                    )
                    .setReversed()
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(50.182, 93.474)),
                                    mirror(new Pose(53.564, 57.274)),
                                    mirror(new Pose(18.039, 58.332))
                            )
                    ).setConstantHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();
        /*.setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(270)),
                            mirrorHeading(Math.toRadians(180))
                    )
                    .build();*/

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(18.039, 58.332)),
                                    mirror(new Pose(70.744, 69.070)),
                                    mirror(new Pose(15.887, 70.660))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(180)),
                            mirrorHeading(Math.toRadians(90))
                    )
                    .setReversed()
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(15.887, 70.660)),
                                    mirror(new Pose(19.444, 70.733)),
                                    mirror(new Pose(36.932, 79.597))
                            )
                    ).setConstantHeadingInterpolation(
                            mirrorHeading(Math.toRadians(90))
                    )
                    .build();
            /*.setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(-90)),
                            mirrorHeading(Math.toRadians(-90))
                    )

                    .build();

             */

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    mirror(new Pose(36.932, 79.597)),
                                    mirror(new Pose(39.711, 80.935)),
                                    mirror(new Pose(48.767, 92.721))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(-90)),
                            mirrorHeading(Math.toRadians(140))
                    )
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    mirror(new Pose(48.767, 92.721)),
                                    mirror(new Pose(21.860, 99.698))
                            )
                    ).setLinearHeadingInterpolation(
                            mirrorHeading(Math.toRadians(140)),
                            mirrorHeading(Math.toRadians(143))
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
            return follower.isBusy(); //returns true when path is complete
        }
    }
    public SequentialTask buildMainTask (Paths paths){
        return new SequentialTask(
                robotContext,

                robotContext.SHOOTER.new setVelocity(robotContext, 0.68),
                new goToPath(paths.Path1),
                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                new WaitTask(robotContext, 1),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                new goToPath(paths.Path2),
//                robotContext.INTAKE.new SetIntakePower(robotContext, 0),

                robotContext.SHOOTER.new setVelocity(robotContext, 0.68),
                new goToPath(paths.Path3),
                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                new goToPath(paths.Path4),
//                robotContext.INTAKE.new SetIntakePower(robotContext, 0),

//                robotContext.SHOOTER.new setVelocity(robotContext, 0.68),
                new goToPath(paths.Path5),
//                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
//                robotContext.TRANSFER.new SendThreeTask(robotContext),
//                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),

//                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                new goToPath(paths.Path6),
                robotContext.SHOOTER.new setVelocity(robotContext, 0.68),
                new WaitTask(robotContext, 0.5),
                robotContext.SHOOTER.new setVelocity(robotContext, Shooter.IDLE_VEL)//,
//                robotContext.INTAKE.new SetIntakePower(robotContext, 0),

//                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
//                new goToPath(paths.Path7),
//                new WaitTask(robotContext, 2),
//
//                robotContext.SHOOTER.new setVelocity(robotContext, 0.68),
//                new goToPath(paths.Path8),
//                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
//                robotContext.TRANSFER.new SendThreeTask(robotContext),
//                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),
//                robotContext.INTAKE.new SetIntakePower(robotContext, 0),

//                new goToPath(paths.Path9)
        );
    }
}
