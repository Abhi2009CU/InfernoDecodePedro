package org.firstinspires.ftc.teamcode.robotStates;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Turret;

public class IntakingState implements State {
    private final MyRobot robotContext;
    private final Task mainTask;

    public IntakingState(MyRobot robotContext) {
        this.robotContext = robotContext;

        mainTask = new SequentialTask(robotContext,
                robotContext.SHOOTER.new setVelocity(robotContext, Shooter.IDLE_VELOCITY),
                robotContext.INTAKE.new ManualRunIntakeMotor(robotContext)
        );
    }

    @Override
    public State step() {

        Gamepad gamepad1 = robotContext.gamepad1;

        follower.setTeleOpDrive(
                Math.pow(-gamepad1.left_stick_y, 3),
                Math.pow(-gamepad1.left_stick_x, 3),
                Math.pow(-gamepad1.right_stick_x, 3),
                false // field Centric
        );

        if (mainTask.step()) {
            return this;
        }

        return new ShootingState(robotContext);
    }

    @Override
    public String getName() {
        return "Intaking";
    }
}
