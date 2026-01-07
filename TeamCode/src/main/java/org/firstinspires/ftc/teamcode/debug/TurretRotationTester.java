package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="Debug - Turret Rotation Tester", group="Debug")
public class TurretRotationTester extends LinearOpMode {
    public static double robotX = 72;
    public static double robotY = 72;
    public static double robotR = 0.5 * Math.PI;
    public static double targetX = 144;
    public static double targetY = 144;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            turret.getCurrentRotation();
            turret.setRotation(Turret.calculateGoalRotation( robotX, robotY, robotR, targetX, targetY));
            turret.updatePID();

            telemetry.addData("Current Rotation (rad)", turret.getCurrentRotation());
            telemetry.addData("Target Rotation (rad)", Turret.calculateGoalRotation( robotX, robotY, robotR, targetX, targetY));
            telemetry.update();
        }
    }
}