package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RisingEdgeTrigger {

    private BooleanSupplier risingEdge;
    private BooleanSupplier otherCondition;

    public RisingEdgeTrigger(BooleanSupplier risingEdge, BooleanSupplier otherCondition) {
        this.risingEdge = risingEdge;
        this.otherCondition = otherCondition;
    }

    public void whileTrue(Command command) {
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(
            new Runnable() {
                private boolean previousEdge = risingEdge.getAsBoolean();
                // private boolean previousOther = otherCondition.getAsBoolean();

                @Override
                public void run() {
                    boolean currentEdge = risingEdge.getAsBoolean();
                    boolean currentOther = otherCondition.getAsBoolean();

                    if (!previousEdge && currentEdge && currentOther) {
                        command.schedule();
                    } else if (!currentEdge) {
                        command.cancel();
                    }

                    previousEdge = currentEdge;
                    // previousOther = currentOther; 
                }
            }
        );
    }
}
