package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subSystems.Shooter;

@Configurable
@TeleOp(name="Hood Tester", group="Debug")
public class HoodTester extends LinearOpMode {
    public static double TARGET_VELOCITY = 0.0;
    public static double HOOD_POSITION = 0.5;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            shooter.setVel(TARGET_VELOCITY);
            shooter.updatePID();
            shooter.setHoodPosition(HOOD_POSITION);

            telemetryM.debug("Current Velocity (ticks/s)", shooter.getVelocity());
            telemetryM.debug("Target Velocity (ticks/s)", TARGET_VELOCITY);

            telemetryM.update();
        }
    }
}