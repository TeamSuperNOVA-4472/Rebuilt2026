package frc.robot.Commands.TeleopCommands;

import java.lang.reflect.Constructor;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.FieldMathHelpers.Location;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem.SpindexerMode;

public class SpindexerTeleop extends Command {
    private final SpindexerSubsystem kSpindexer;
    private final SpindexerMode kNewMode;
    private Supplier<Boolean> kTurretAtSetpoint;
    private Supplier<Location> kLocation;
    
    public SpindexerTeleop(
        SpindexerSubsystem mSpindexer, 
        SpindexerMode mNewMode, 
        Supplier<Boolean> mTurretAtSetpoint, 
        Supplier<Location> mLocation){ 
        kSpindexer = mSpindexer;
        kNewMode = mNewMode;
        kTurretAtSetpoint = mTurretAtSetpoint;
        kLocation = mLocation;
    }

    @Override
    public void execute() {
        SpindexerMode mode;
        // TODO: is different kicker speed still needed with spindexer fixes? 
        switch (kLocation.get()) // Get location
        {
            case ALLIANCE_ZONE: // Adjust for hub distance
                if (kTurretAtSetpoint.get())
                {
                    mode = kNewMode;
                }
                else
                {
                    mode = SpindexerMode.OFF;
                }
                break;
            default: mode = kNewMode;
        }

        kSpindexer.setMode(mode);
    }
}
