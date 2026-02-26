package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class MoveTurretAbsolute extends InstantCommand{
    private final TurretSubsystem kTurret;
    private final Supplier<Double> kGetHeadingDegrees;
    private final Supplier<Double> kAbsTargetAngle;

    public MoveTurretAbsolute(TurretSubsystem mTurret, Supplier<Double> mGetHeadingDegrees, Supplier<Double> mAbsTargetAngle)
    {
        kAbsTargetAngle = mAbsTargetAngle;
        kGetHeadingDegrees = mGetHeadingDegrees;
        kTurret = mTurret;

        addRequirements(kTurret);
    }

    @Override
    public void execute() {
        double kRelativeAngle = kGetHeadingDegrees.get() + kAbsTargetAngle.get() + 179;
        double kConstrainedAngle = (kRelativeAngle % 360 + 360) % 360;
        kTurret.setTargetAngle(kConstrainedAngle);
    }

}
