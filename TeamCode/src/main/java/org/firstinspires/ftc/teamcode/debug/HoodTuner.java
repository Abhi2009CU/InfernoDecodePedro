package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Shooter;

@Configurable

@TeleOp(name="Hood Tuner", group="Debug")
public class HoodTuner extends LinearOpMode {
    public static double HOOD_POSITION = 0;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            shooter.setHoodPosition(HOOD_POSITION);

            telemetryM.debug("Hood Position", HOOD_POSITION);

            telemetryM.update();
        }
    }
}