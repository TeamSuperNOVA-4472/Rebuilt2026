package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    public static final SpindexerSubsystem kSpindexer = new SpindexerSubsystem();

    public enum SpindexerMode{
        OFF,
        LOAD
    }
    private TalonFX kToFlywheelMotor;
    private SpindexerMode kMode;

    private SpindexerSubsystem(){;
        kToFlywheelMotor = new TalonFX(24);
    }

    private void moveSpindexer(){
        switch (kMode){
        case OFF:
            kToFlywheelMotor.set(0);
            break;

        case LOAD:
            kToFlywheelMotor.set(Constants.SpindexerConstants.kSpindexerSpeed);
            break;
        }
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