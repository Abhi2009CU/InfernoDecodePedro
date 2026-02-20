package org.firstinspires.ftc.teamcode.debug;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MainTeleOp;
import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="Shoot Point Test", group="Debug")
public class ShootPointTest extends LinearOpMode {
    private final Pose START_POSE = new Pose(57, 8.5, 0.5 * Math.PI);

    public static double TARGET_HOOD_POS = 0;
    public static double TARGET_VEL = 0;

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

        waitForStart();

        robotContext.TURRET.resetEncoder();

        follower.setStartingPose(START_POSE);

        double targetX = 9;
        double targetY = 140;

        follower.update();
        follower.startTeleopDrive(true);

        Task controlTask = getControlTask(robotContext);

        while (opModeIsActive()){
            follower.update();

            Pose currentPose = follower.getPose();

            robotContext.TURRET.setRotation(Turret.calculateGoalRotation(currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            robotContext.TURRET.updatePID();
            robotContext.SHOOTER.updatePID();

            robotContext.SHOOTER.setHoodPosition(TARGET_HOOD_POS);
            robotContext.SHOOTER.setVel(TARGET_VEL);

            controlTask.step();

            if (gamepad1.triangle) {
                follower.setPose(new Pose(27, 132, 2.51327));
            }

            if (gamepad1.dpad_right) {
                robotContext.TURRET.incrementAngleOffset(0.01);
            }
            if (gamepad1.dpad_left) {
                robotContext.TURRET.incrementAngleOffset(-0.01);
            }
            if(gamepad1.square) {
                robotContext.TURRET.setAngleOffset(0);
            }

            follower.setTeleOpDrive(
                    Math.pow(-gamepad1.left_stick_y, 3),
                    Math.pow(-gamepad1.left_stick_x, 3),
                    Math.pow(-gamepad1.right_stick_x, 3),
                    true // robot Centric
            );

            double d = Math.sqrt( Math.pow(currentPose.getX() - targetX, 2) + Math.pow(currentPose.getY() - targetY, 2) );

            telemetryM.addData("Distance", d);
            telemetryM.addData("Target hood pos", TARGET_HOOD_POS);
            telemetryM.addData("Target vel", TARGET_VEL);

            telemetryM.addData("Actual vel", robotContext.SHOOTER.getVelocity());

            telemetryM.update();
        }
    }

    @NonNull
    private Task getControlTask(MyRobot robotContext) {
        return new ParallelTask(robotContext, false,
                robotContext.INTAKE.new ManualRunIntakeMotor(robotContext),
                robotContext.TRANSFER.new TransferTask(robotContext)
        );
    }
}