package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="Turret Rotation Debugger", group="Debug")
public class TurretRotationTeleOp extends LinearOpMode {
    public static double robotX = 72;
    public static double robotY = 72;
    public static double robotR = 0;
    public static double targetX = 144;
    public static double targetY = 144;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            turret.getCurrentRotation();
            turret.setRotation(robotX, robotY, robotR, targetX, targetY);
            turret.updatePID();
        }
    }
}