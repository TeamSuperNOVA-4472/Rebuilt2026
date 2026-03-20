package frc.robot.Commands;

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

public class setSpindexer extends Command {
    private final SpindexerSubsystem kSpindexer;
    private final SpindexerMode kNewMode;
    private Supplier<Double> kDistance;
    private Supplier<Boolean> kTurretAtSetpoint;
    private Supplier<Location> kLocation;
    
    public setSpindexer(
        SpindexerSubsystem mSpindexer, 
        SpindexerMode mNewMode, 
        Supplier<Boolean> mTurretAtSetpoint, 
        Supplier<Double> mDistance,
        Supplier<Location> mLocation){ 
        kSpindexer = mSpindexer;
        kNewMode = mNewMode;
        kDistance = mDistance;
        kTurretAtSetpoint = mTurretAtSetpoint;
        kLocation = mLocation;
    }

    @Override
    public void execute() {
        if (kTurretAtSetpoint.get()) // Only shoot if turret is at the setpoint
        {
            switch (kLocation.get()) // Get location
            {
                case ALLIANCE_ZONE: // Adjust for hub distance
                    double distance = MathUtil.clamp(kDistance.get(), FlywheelConstants.kDistanceMinimumInMeters, FlywheelConstants.kDistanceMaximumInMeters);
                    double kickerSpeed = SpindexerConstants.kDistanceToKickerSpeed.get(distance);
                    kSpindexer.setKickerVelocity(kickerSpeed);
                    break;
                default: kSpindexer.setKickerVelocity(SpindexerConstants.kKickerSpeed);
            }
            kSpindexer.setMode(kNewMode);
        }
        else
        {
            kSpindexer.setMode(SpindexerMode.OFF);
        }
    }
}
