package org.firstinspires.ftc.teamcode;

import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

/**
 * MyRobot class that extends RobotContext to include robot-specific subsystems.
 */
public class MyRobot extends RobotContext {
    public static Follower follower = null;

    public final Intake INTAKE;
    public final Shooter SHOOTER;
    public final Transfer TRANSFER;
    public final Turret TURRET;

    //not sure about the following
    public Gamepad gamepad1, gamepad2;

    /**
     * Creates a new RobotContext with the specified telemetry and gamepad references.
     * All parameters are required and cannot be null.
     *
     * @param telemetry the telemetry instance for driver station communication
     * @param gamepad1  the primary gamepad controller
     * @param gamepad2  the secondary gamepad controller
     */
    public MyRobot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, Intake intake, Shooter shooter, Transfer transfer, Turret turret) {
        super(telemetry, gamepad1, gamepad2);
        this.INTAKE = intake;
        this.SHOOTER = shooter;
        this.TRANSFER = transfer;
        this.TURRET = turret;

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
}