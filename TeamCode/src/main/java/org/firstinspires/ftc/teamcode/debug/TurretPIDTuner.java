package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="Debug - Turret PID Tuner", group="Debug")
public class TurretPIDTuner extends LinearOpMode {
    public static double TARGET_ROTATION = 0.0;

    @Override
    public void runOpMode() {
        TARGET_ROTATION = 0.0;
        Turret turret = new Turret(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                TARGET_ROTATION = Math.PI / 2;
            } else if (gamepad1.b) {
                TARGET_ROTATION = -Math.PI / 2;
            }

            turret.setRotation(TARGET_ROTATION);
            turret.updatePID();

            telemetry.addData("Current Rotation (rad)", turret.getCurrentRotation());
            telemetry.addData("Target Rotation (rad)", TARGET_ROTATION);
            telemetry.update();
        }
    }
}