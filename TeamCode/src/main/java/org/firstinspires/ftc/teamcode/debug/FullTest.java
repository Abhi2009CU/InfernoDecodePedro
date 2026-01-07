package org.firstinspires.ftc.teamcode.debug;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="Full Test", group="Debug")
public class FullTest extends LinearOpMode {
    public static double HOOD_POSITION = 0.45;

    public static double targetX = 135;
    public static double targetY = 135;

    public static Pose startingPose = new Pose(72, 72, 0.5 * Math.PI);

    public static double LEFT_UP = 0.5;
    public static double LEFT_DOWN = 0.15;

    public static double RIGHT_UP = 0.49;
    public static double RIGHT_DOWN = 0.83;

    private TelemetryManager telemetryM;

    private Servo left;
    private Servo right;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        Shooter shooter = new Shooter(hardwareMap);

        Motor intakeMotor = new Motor(hardwareMap, "Intake");
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        left = hardwareMap.get(Servo.class, "LeftFlap");
        right = hardwareMap.get(Servo.class, "RightFlap");


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            shooter.setHoodPosition(HOOD_POSITION);

            shooter.updatePID();

            double intakePower = Math.max(-1.0, Math.min(1.0, gamepad1.right_trigger - gamepad1.left_trigger));
            intakeMotor.set(intakePower);

            if (gamepad1.right_bumper || gamepad1.a) {
                right.setPosition(RIGHT_UP);
            } else {
                right.setPosition(RIGHT_DOWN);
            }

            if (gamepad1.left_bumper || gamepad1.a) {
                left.setPosition(LEFT_UP);
            } else {
                left.setPosition(LEFT_DOWN);
            }

            if (gamepad2.dpad_up) {
                shooter.setVel(1);
            } else if (gamepad2.dpad_down) {
                shooter.setVel(0);
            }

            if (gamepad2.right_bumper) {
                turret.incrementAngleOffset(0.01);
            }
            if (gamepad2.left_bumper) {
                turret.incrementAngleOffset(-0.01);
            }

            follower.update();

            follower.setTeleOpDrive(
                    Math.pow(-gamepad1.left_stick_y, 3),
                    Math.pow(-gamepad1.left_stick_x, 3),
                    Math.pow(-gamepad1.right_stick_x, 3),
                    false // field Centric
            );


            Pose currentPose = follower.getPose();

            turret.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            turret.updatePID();

            telemetryM.debug("Current Velocity (ticks/s)", shooter.getVelocity());
            telemetryM.debug("Current Distance", Math.sqrt(Math.pow(currentPose.getX() - targetX, 2) + Math.pow(currentPose.getY() - targetY, 2)));

            telemetryM.update();
        }
    }
}