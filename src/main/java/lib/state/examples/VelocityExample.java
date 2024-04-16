package lib.state.examples;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import lib.log.Loggable;
import lib.state.VelocityState;
import lib.state.machines.VelocityStateController;


public class VelocityExample extends VelocityStateController implements Loggable {

    private final VelocityState shoot =  DummyConstants.VelocityExample.shoot;
    private final VelocityState shootRamp = DummyConstants.VelocityExample.shootRamp;
    private final VelocityState idle = DummyConstants.VelocityExample.idle;

    public VelocityExample(){
        super(DummyConstants.VelocityExample.idle);
    }
    @Override
    public void log() {
        SmartDashboard.putNumber("foo", shoot.getTalonSetup().motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {}

    public Command shootCommand() {
        return new Command() {
            @Override
            public void initialize() {
                setState(shootRamp);
            }

            @Override
            public void execute() {
                if(isAtState(shootRamp)){
                    setState(shoot);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setState(idle);
            }
        };
    }
}
