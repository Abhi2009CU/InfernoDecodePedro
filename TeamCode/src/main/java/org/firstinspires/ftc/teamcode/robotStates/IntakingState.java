package org.firstinspires.ftc.teamcode.robotStates;


import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;

public class IntakingState implements State {
    private final MyRobot robotContext;
    private final Task mainTask;

    private boolean rumbled = false;


    public IntakingState(MyRobot robotContext) {
        this.robotContext = robotContext;

        mainTask = new SequentialTask(robotContext,
                new ParallelTask(robotContext, true,
                        robotContext.TRANSFER.new ManualControlTask(robotContext),
                        robotContext.INTAKE.new ManualRunIntakeMotor(robotContext)
                )
        );
    }

    @Override
    public State step() {
        if(!rumbled) {
            rumbled = true;
            robotContext.GAMEPAD1.rumbleBlips(1);
            robotContext.GAMEPAD2.rumbleBlips(1);
        }

        robotContext.SHOOTER.setVel(Shooter.IDLE_VEL);

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
