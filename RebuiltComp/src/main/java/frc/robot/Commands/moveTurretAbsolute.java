package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class moveTurretAbsolute extends InstantCommand{
    private TurretSubsystem kTurret;
    private SwerveSubsystem kSwerveSubsystem;
    private double kAbsTargetAngle;


    public moveTurretAbsolute(TurretSubsystem mTurret, SwerveSubsystem mSwerveSubsystem, double mAbsTargetAngle)
    {
        kAbsTargetAngle = mAbsTargetAngle;
        kSwerveSubsystem = mSwerveSubsystem;
        kTurret = mTurret;
        addRequirements(kTurret);
    }
    @Override
    public void initialize(){
        double kRelativeAngle = kSwerveSubsystem.getHeadingDegrees() + kAbsTargetAngle;
        double kConstrainedAngle = (kRelativeAngle % 360 + 360) % 360;
    }

}
