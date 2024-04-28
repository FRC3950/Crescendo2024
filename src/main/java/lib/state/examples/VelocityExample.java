package lib.state.examples;

import edu.wpi.first.wpilibj2.command.Command;
import lib.log.Loggable;
import lib.meta.CommandBehavior;
import lib.meta.CommandType;
import lib.meta.EndType;
import lib.meta.EndsOn;
import lib.state.VelocityState;
import lib.state.machines.VelocityStateMachine;

import java.util.HashSet;


public class VelocityExample extends VelocityStateMachine implements Loggable {

    private final HashSet<VelocityState> shoot = DummyConstants.VelocityExample.shoot;
    private final HashSet<VelocityState> idle = DummyConstants.VelocityExample.idle;
    private final HashSet<VelocityState> shootRamp = DummyConstants.VelocityExample.shootRamp;


    public VelocityExample(){
        super(DummyConstants.VelocityExample.idle);
    }

    @Override
    public void log() {
        System.out.println("Hello!");
    }

    @CommandBehavior(behavior = CommandType.SUSTAINED_EXECUTE)
    @EndsOn(endsOn = EndType.INTERRUPT)
    public Command shootCommand() {
        return new Command() {
            @Override
            public void initialize() {
                acquireGoalState(shootRamp);
            }

            @Override
            public void execute() {
                if(isAtState(shootRamp)){
                    acquireGoalState(shoot);
                }
            }

            @Override
            public void end(boolean interrupted) {
                acquireGoalState(idle);
            }
        };
    }
}
