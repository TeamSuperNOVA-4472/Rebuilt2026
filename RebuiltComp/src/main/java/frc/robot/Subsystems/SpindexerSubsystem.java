package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    public static final SpindexerSubsystem kInstance = new SpindexerSubsystem();
     public static final double kSpindexerSpeed = 0.9;

    public enum SpindexerMode{
        OFF,
        LOAD
    }
    private TalonFX kSpindexterMotor;
    private TalonFX kToFlywheelMotor;
    private SpindexerMode kMode;

    private SpindexerSubsystem(){
        kMode = SpindexerMode.OFF;
        kSpindexterMotor = new TalonFX(7);
        kToFlywheelMotor = new TalonFX(6);
    }

    private void moveSpindexer(){
        switch (kMode){
        case OFF:
            kSpindexterMotor.set(0);
            kToFlywheelMotor.set(0);
            break;

        case LOAD:
            kSpindexterMotor.set(kSpindexerSpeed);
            kToFlywheelMotor.set(kSpindexerSpeed);
            break;
        }
    }
    public double getSpinSpeed(){
        return kSpindexterMotor.get();
    }
    public SpindexerMode getMode(){
        return kMode;
    }
    public void setMode(SpindexerMode mNewMode){
        kMode = mNewMode;
        SmartDashboard.putNumber("Spindexer Mode", kMode.ordinal());
        moveSpindexer();
    }
}