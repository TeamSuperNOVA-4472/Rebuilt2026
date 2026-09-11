package frc.robot.Commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Subsystems.FlywheelSubsystem;
import frc.robot.Subsystems.FlywheelSubsystem.FlywheelMode;

public class setFlywheelSlowAuto extends Command {
    private final FlywheelSubsystem kFlywheel;

    public setFlywheelSlowAuto(FlywheelSubsystem mFlywheelSubsystem){
        kFlywheel = mFlywheelSubsystem;

        addRequirements(kFlywheel);
    }

    @Override
    public void initialize(){
        kFlywheel.disableFlywheelBangBang();
        kFlywheel.setMode(FlywheelMode.SPINNING, FlywheelConstants.kSlowSpeed);
        kFlywheel.setHoodTarget(FlywheelConstants.kStartingHoodAngle);
    }

    @Override
    public boolean isFinished(){
        return kFlywheel.getFlywheelAtTarget();
    }

    @Override
    public void end(boolean isInterupted){
        kFlywheel.enableFlywheelBangBang();
    }
}
