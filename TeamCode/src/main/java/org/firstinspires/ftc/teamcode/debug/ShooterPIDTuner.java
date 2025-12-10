package org.firstinspires.ftc.teamcode.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

@Configurable
@TeleOp(name="Shooter PID Tuner", group="Debug")
public class ShooterPIDTuner extends LinearOpMode {
    public static double TARGET_VELOCITY = 0.0;

    private TelemetryManager telemetryM;

    private Servo left;
    private Servo right;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        left = hardwareMap.get(Servo.class, "LeftFlap");
        right = hardwareMap.get(Servo.class, "RightFlap");


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            shooter.setVel(TARGET_VELOCITY);
            shooter.updatePID();

            if (gamepad1.a) {
                left.setPosition(0.9);
                right.setPosition(0.2);
            } else {
                left.setPosition(0.2);
                right.setPosition(0.9);
            }

            telemetryM.addData("Current Velocity", shooter.getVelocity());
            telemetryM.addData("Target Velocity", TARGET_VELOCITY);


            telemetryM.update();
        }
    }
}