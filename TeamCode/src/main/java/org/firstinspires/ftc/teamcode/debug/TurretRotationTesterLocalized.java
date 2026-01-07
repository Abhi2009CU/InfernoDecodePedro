package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="Debug - Turret Rotation Tester Localized", group="Debug")
public class TurretRotationTesterLocalized extends LinearOpMode {
    public static double targetX = 144;
    public static double targetY = 144;

    public static Pose startingPose = new Pose(72, 72, 0.5 * Math.PI);

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            follower.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // robot Centric
            );

            Pose currentPose = follower.getPose();

            turret.getCurrentRotation();
            turret.setRotation(Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            turret.updatePID();

            telemetry.addData("Current Rotation (rad)", turret.getCurrentRotation());
            telemetry.addData("Target Rotation (rad)", Turret.calculateGoalRotation( currentPose.getX(), currentPose.getY(), currentPose.getHeading(), targetX, targetY));
            telemetry.update();
        }
    }
}