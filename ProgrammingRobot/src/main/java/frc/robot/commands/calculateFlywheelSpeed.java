package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem.FlywheelMode;

public class calculateFlywheelSpeed extends Command {
    private FlywheelSubsystem kFlywheel;
    private SwerveSubsystem kSwerve;
    private FlywheelMode kFlyMode;

    public calculateFlywheelSpeed(FlywheelSubsystem mFlywheel, SwerveSubsystem mSwerve, FlywheelMode mFlyMode){
        kFlywheel = mFlywheel;
        kFlyMode = mFlyMode;
    }

    @Override
    public void initialize(){
        kFlywheel.setMode(kFlyMode);
        final double gravity = 9.81;
        final double angle = 0.69813;
        final double parallelSpeed = 0;
        final double height = 0.99695;
        final double dis = 4.445;
        final double outVel = (Math.pow(1/Math.cos(angle),2)*
        Math.sqrt(2*(Math.pow(Math.cos(angle)*gravity*dis, 2)*
        (Math.sin(angle)*Math.cos(angle)*gravity*dis+
        Math.pow(Math.sin(angle)*parallelSpeed,2)/2-
        Math.pow(Math.cos(angle),2)*gravity*height)))-
        parallelSpeed*
        (gravity*dis*Math.tan(angle)-
        2*gravity*height))/
        (2*gravity*dis*Math.sin(angle)-2*gravity*height*Math.cos(angle));
        final double MOIFuel = 0.00048375;
        final double fuelRad = 0.075;
        final double flyRad = 0.0508;
        final double fuelMass = 0.215;
        final double flyVel = (outVel+((MOIFuel*outVel)/(fuelMass*Math.pow(fuelRad,2))))/flyRad;
        kFlywheel.setTargetSpeed(flyVel/(2*Math.PI)*1.82);
    }
    @Override
    public boolean isFinished(){
        return kFlywheel.getMode() == FlywheelMode.OFF || Math.abs(kFlywheel.getSpinSpeed()-kFlywheel.getTargetSpeed()) < 0.5;
    }
}