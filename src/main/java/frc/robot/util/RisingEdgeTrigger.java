package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public record RisingEdgeTrigger(BooleanSupplier risingEdge, BooleanSupplier otherCondition, boolean stopOnOtherFalse) {
    public RisingEdgeTrigger(BooleanSupplier risingEdge, BooleanSupplier otherCondition) {
        this(risingEdge, otherCondition, false);
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
                        } else if (!currentEdge || (!currentOther && stopOnOtherFalse)) {
                            command.cancel();
                        }

                        previousEdge = currentEdge;
                        // previousOther = currentOther;
                    }
                }
        );
    }

    /**
     * @param onTrue  The command to run when the trigger is true
     * @param onFalse The command to run when the trigger is false
     */
    public void onTrueOnFalse(Command onTrue, Command onFalse) {
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(
                new Runnable() {
                    private boolean previousEdge = risingEdge.getAsBoolean();
                    private boolean previousOther = otherCondition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean currentEdge = risingEdge.getAsBoolean();
                        boolean currentOther = otherCondition.getAsBoolean();

                        if (!previousEdge && currentEdge && currentOther) onTrue.schedule();
                        if ((previousEdge && previousOther) && (!currentEdge || !currentOther)) onFalse.schedule();

                        previousEdge = currentEdge;
                        previousOther = currentOther;
                    }
                }
        );
    }

    public void onTrue(Command onTrue) {
        onTrueOnFalse(onTrue, Commands.none());
    }

    public void onFalse(Command onFalse) {
        onTrueOnFalse(Commands.none(), onFalse);
    }

    public RisingEdgeTrigger and(BooleanSupplier other) {
        return new RisingEdgeTrigger(risingEdge, () -> otherCondition.getAsBoolean() && other.getAsBoolean());
    }
}
