package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public static final IntakeSubsystem kIntake = new IntakeSubsystem();

    enum IntakeMode{
        STORED,
        INTAKE,
        OUTTAKE
    }
    private IntakeMode kMode;

    private boolean kIsAtState;
    private double kSliderTarget; 

    private TalonFX kIntakeMotor;
    private TalonFX kIntakeSlider;
    private PIDController kSliderPID;

    private IntakeSubsystem(){
        kMode = IntakeMode.STORED;
        kIsAtState = true;
        kIntakeMotor = new TalonFX(IntakeSubsystemConstants.kIntakeMotorPort);
        kIntakeSlider = new TalonFX(IntakeSubsystemConstants.kSliderMotorPort);
    }

    private void moveToState(){
        switch (kMode){
        case STORED:
            kSliderTarget = IntakeSubsystemConstants.kStoredPos;
            kIntakeMotor.set(0);
            break;
        
        case INTAKE:
            kSliderTarget = IntakeSubsystemConstants.kOutPos;
            kIntakeMotor.set(IntakeSubsystemConstants.kIntakeMotorSpeed);
            break;
        
        case OUTTAKE:
            kSliderTarget = IntakeSubsystemConstants.kOutPos;
            kIntakeMotor.set(-IntakeSubsystemConstants.kIntakeMotorSpeed);
            break;
        
        }
        
        

        //desired target for the slider, speed of intake motor
    }
    
    public void setIntake(IntakeMode mNewMode){
        kMode = mNewMode;
        moveToState();
    }
    public IntakeMode getMode(){
        return kMode;
    }
    public boolean isReady(){
        return kIsAtState;
    }
    public double getIntakeSpeed(){
        return kIntakeMotor.get();
    }

    @Override
    public void periodic(){}
}
