package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {
    public static final SpindexerSubsystem kInstance = new SpindexerSubsystem();

    public enum SpindexerMode{
        OFF,
        LOAD
    }
    private TalonFX kSpindexterMotor;
    private TalonFX kToFlywheelMoter;
    private SpindexerMode kMode;

    private SpindexerSubsystem(){
        kMode = SpindexerMode.OFF;
        kSpindexterMotor = new TalonFX(7);
        kToFlywheelMoter = new TalonFX(6);
    }

    private void moveSpindexer(){}
    public double getSpinSpeed(){
        return kSpindexterMotor.get();
    }
    public SpindexerMode getMode(){
        return kMode;
    }
    public void setMode(SpindexerMode mNewMode){
        kMode = mNewMode;
        moveSpindexer();
    }
}
