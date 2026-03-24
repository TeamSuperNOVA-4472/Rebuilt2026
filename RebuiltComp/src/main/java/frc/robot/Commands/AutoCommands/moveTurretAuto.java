package frc.robot.Commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class moveTurretAuto extends InstantCommand{
    private final TurretSubsystem kTurret;
    private final Supplier<Double> kGetHeadingDegrees;
    private final Supplier<Double> kAbsTargetAngle;

    public moveTurretAuto(TurretSubsystem mTurret, Supplier<Double> mGetHeadingDegrees, Supplier<Double> mAbsTargetAngle)
    {
        kAbsTargetAngle = mAbsTargetAngle;
        kGetHeadingDegrees = mGetHeadingDegrees;
        kTurret = mTurret;

        addRequirements(kTurret);
    }

    @Override
    public void initialize() {
        double kRelativeAngle = kGetHeadingDegrees.get() - kAbsTargetAngle.get() + TurretConstants.kStartingAngleOffset;
        double kConstrainedAngle = (kRelativeAngle % 360 + 360) % 360;
        kTurret.setTargetAngle(kConstrainedAngle);
        SmartDashboard.putNumber("Absolute Angle", kAbsTargetAngle.get());
    }

}
