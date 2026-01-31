package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {
    public static final SpindexerSubsystem kInstance = new SpindexerSubsystem();

    public enum SpindexerMode{

    }
    private SpindexerMode kMode;

    private SpindexerSubsystem(){

    }

    private void moveSpindexer(){}
    public double getSpinSpeed(){}
    public SpindexerMode getMode(){}
    public void setMode(SpindexerMode mNewMode){}
}
