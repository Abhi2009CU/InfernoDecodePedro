package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

import java.util.Timer;

@Configurable
@Autonomous(name="AUTON_DRIVE_FORWARD", group="Linear OpMode")
public class DriveForwardAuton extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE
    }

    public static double BLUE_STARTING_X = 87;
    public static double BLUE_STARTING_Y = 8.5;
    public static double BLUE_STARTING_HEADING = 0.5 * Math.PI;

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

        follower.update();

        follower.startTeleopDrive(true);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()){
            follower.update();

            if (timer.seconds() < 0.5) {
                follower.setTeleOpDrive(
                        1,
                        0,
                        0,
                        true
                );
            } else {
                follower.setTeleOpDrive(
                        0,
                        0,
                        0,
                        true
                );
            }
        }
    }
}