package frc.robot.Commands;

import java.lang.reflect.Constructor;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;

public class setSpindexer extends Command {
    private final SpindexerSubsystem kSpindexer;
    private final SpindexerMode kNewMode;
    private Supplier<Double> kDistance;
    private Supplier<Boolean> kTurretAtSetpoint;
    
    public setSpindexer(SpindexerSubsystem mSpindexer, SpindexerMode mNewMode, Supplier<Boolean> mTurretAtSetpoint, Supplier<Double> mDistance){ 
        kSpindexer = mSpindexer;
        kNewMode = mNewMode;
        kDistance = mDistance;
        kTurretAtSetpoint = mTurretAtSetpoint;
    }

    @Override
    public void execute() {
        // TODO: add location specific behavior for passing
        if (kTurretAtSetpoint.get())
        {
            if (kDistance.get() <= FlywheelConstants.kDistanceMaximumInMeters && kDistance.get() >= FlywheelConstants.kDistanceMinimumInMeters)
            {
                kSpindexer.setKickerVelocity(SpindexerConstants.kDistanceToKickerSpeed.get(kDistance.get()));
            }
            else
            {
                kSpindexer.setKickerVelocity(SpindexerConstants.kKickerSpeed);
            }
            kSpindexer.setMode(kNewMode);
        }
        else
        {
            kSpindexer.setMode(SpindexerMode.OFF);
        }
    }
}
