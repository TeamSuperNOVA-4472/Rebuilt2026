package frc.robot.Subsystems;

import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GameDataHelpers;
import frc.robot.Constants.GameConstants;

public class OperatorFeedbackSubsystem extends SubsystemBase {
    private final XboxController kDriver;
    private final XboxController kOperator;

    public OperatorFeedbackSubsystem(XboxController mDriver, XboxController mOperator)
    {
        kDriver = mDriver;
        kOperator = mOperator;
    }

}
