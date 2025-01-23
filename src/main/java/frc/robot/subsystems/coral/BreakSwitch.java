package frc.robot.subsystems.coral;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ConveyorConstants;

public class BreakSwitch {
    private DigitalInput initialConveyorSensor;
    public void Conveyor() {


    initialConveyorSensor = new DigitalInput(ConveyorConstants.initialConveyorSensor);
  }
  public boolean getInitialConveyorSensor() {
    return !initialConveyorSensor.get();
  }

}
