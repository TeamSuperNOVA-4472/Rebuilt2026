package frc.robot.Subsystems;

import static frc.robot.Constants.SwerveConstants.kA;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.IntakeSubsystemConstants;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;

public class IntakeSubsystem extends SubsystemBase {
    
    public enum IntakeStorageMode{
        STORED,
        OUT
    }

    public enum IntakeActionMode{
        INTAKE,
        OUTTAKE,
        OFF
    }

    // private final Trigger kStatorLimitExceeded;

    private IntakeStorageMode kStorageMode;
    private IntakeActionMode kActionMode;

    private boolean kIsAtState;
    private double kSliderTarget; 
    private double PIDOutput;
    private boolean kPIDEnabled = true;

    private final TalonFX kIntakeMotor;
    private final TalonFX kIntakeSlider;
    private final ProfiledPIDController kSliderPID;

    private final DCMotor kIntakeSimMotor;
    private final ElevatorSim kIntakeSim;
    private final Mechanism2d kSimSpace;
    private final MechanismRoot2d kSimRoot;
    private final MechanismLigament2d kSimDisp;

    public IntakeSubsystem(){
        kStorageMode = IntakeStorageMode.STORED;
        kActionMode = IntakeActionMode.OFF;
        kIsAtState = true;
        kIntakeMotor = new TalonFX(Constants.IntakeSubsystemConstants.kIntakeMotorPort, IntakeSubsystemConstants.kIntakeCanbus);
        kIntakeSlider = new TalonFX(Constants.IntakeSubsystemConstants.kSliderMotorPort, IntakeSubsystemConstants.kSliderCanbus);
        kSliderPID = new ProfiledPIDController(Constants.IntakeSubsystemConstants.kSliderP, Constants.IntakeSubsystemConstants.kSliderI, Constants.IntakeSubsystemConstants.kSliderD, new TrapezoidProfile.Constraints(IntakeSubsystemConstants.kSliderMaxVelocity,IntakeSubsystemConstants.kSliderMaxAcceleration));
        kSliderPID.setTolerance(Constants.IntakeSubsystemConstants.kSlideThreshold, Constants.IntakeSubsystemConstants.kSlideSpeedThreshold);
        kIntakeSimMotor = DCMotor.getKrakenX44(IntakeSubsystemConstants.kSimNumMotors);
        kIntakeSim = new ElevatorSim(kIntakeSimMotor, Constants.IntakeSubsystemConstants.kGearing, Constants.IntakeSubsystemConstants.kMass, Constants.IntakeSubsystemConstants.kDrumRadius, 0, Constants.IntakeSubsystemConstants.kMaxLen, false, 0, 0, 0);
        kSimSpace = new Mechanism2d(IntakeSubsystemConstants.kSimWidth, IntakeSubsystemConstants.kSimHeight);
        kSimRoot = kSimSpace.getRoot(IntakeSubsystemConstants.kSimRootName, IntakeSubsystemConstants.kSimX, IntakeSubsystemConstants.kSimY);
        kSimDisp = kSimRoot.append(new MechanismLigament2d("Intake", kIntakeSim.getPositionMeters()*Constants.IntakeSubsystemConstants.kSimLenMult, Constants.IntakeSubsystemConstants.kIntakeAngle));
        SmartDashboard.putData("IntakeSim", kSimSpace);

        TalonFXConfiguration kIntakeConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kIntakeCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kIntakeMotorConfig = new MotorOutputConfigs();
        kIntakeMotor.getConfigurator().refresh(kIntakeConfig);
        kIntakeMotor.getConfigurator().refresh(kIntakeCurrentConfig);
        kIntakeMotor.getConfigurator().refresh(kIntakeMotorConfig);
        kIntakeCurrentConfig.SupplyCurrentLimit = IntakeSubsystemConstants.kIntakeSupplyLimit;
        kIntakeCurrentConfig.SupplyCurrentLimitEnable = IntakeSubsystemConstants.kIntakeSupplyLimitEnabled;
        kIntakeCurrentConfig.StatorCurrentLimitEnable = IntakeSubsystemConstants.kIntakeStatorLimitEnabled;
        kIntakeCurrentConfig.StatorCurrentLimit = IntakeSubsystemConstants.kIntakeStatorLimit;
        kIntakeMotorConfig.NeutralMode = IntakeSubsystemConstants.kIntakeNeutralMode;
        kIntakeConfig.withCurrentLimits(kIntakeCurrentConfig);
        kIntakeConfig.withMotorOutput(kIntakeMotorConfig);
        kIntakeMotor.getConfigurator().apply(kIntakeConfig);

        TalonFXConfiguration kSliderConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kSliderCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kSliderMotorConfig = new MotorOutputConfigs();
        kIntakeSlider.getConfigurator().refresh(kSliderConfig);
        kIntakeSlider.getConfigurator().refresh(kSliderCurrentConfig);
        kIntakeSlider.getConfigurator().refresh(kSliderMotorConfig);
        kSliderCurrentConfig.SupplyCurrentLimit = IntakeSubsystemConstants.kSliderSupplyLimit;
        kSliderCurrentConfig.SupplyCurrentLimitEnable = IntakeSubsystemConstants.kSliderSupplyLimitEnabled;
        kSliderCurrentConfig.StatorCurrentLimitEnable = IntakeSubsystemConstants.kSliderStatorLimitEnabled;
        kSliderCurrentConfig.StatorCurrentLimit = IntakeSubsystemConstants.kSliderStatorLimit;
        kSliderMotorConfig.NeutralMode = IntakeSubsystemConstants.kSliderNeutralMode;
        kSliderConfig.withCurrentLimits(kSliderCurrentConfig);
        kSliderConfig.withMotorOutput(kSliderMotorConfig);
        kIntakeSlider.getConfigurator().apply(kSliderConfig);
        kIntakeSlider.setPosition(0);

    }

    private void moveToStorageState(){
        switch (kStorageMode){
        case STORED:
            kSliderTarget = Constants.IntakeSubsystemConstants.kStoredPos;
            break;

        case OUT:
            kSliderTarget = Constants.IntakeSubsystemConstants.kOutPos;
            break;
        
        }
    }

    private void setActionState(){
        switch (kActionMode){
        case INTAKE:
            kIntakeMotor.set(-Constants.IntakeSubsystemConstants.kIntakeMotorSpeed);
            break;
        
        case OUTTAKE:
            kIntakeMotor.set(Constants.IntakeSubsystemConstants.kIntakeMotorSpeed);
            break;
        
        case OFF:
            kIntakeMotor.set(0);
            break;
        }
    }
    
    public void setIntakeAction(IntakeActionMode mNewMode){
        kActionMode = mNewMode;
        setActionState();
    }

    public void setIntakeStorage(IntakeStorageMode mNewMode){
        kStorageMode = mNewMode;
        moveToStorageState();
    }

    public void resetSliderEncoderToOutPosition()
    {
        kIntakeSlider.setPosition(IntakeSubsystemConstants.kOutPos/Constants.IntakeSubsystemConstants.kEncoderToInchesMult);
    }

    public double getSliderStator()
    {
        return kIntakeSlider.getStatorCurrent().getValueAsDouble();
    }

    public IntakeActionMode getActionMode(){
        return kActionMode;
    }

    public IntakeStorageMode getStorageMode(){
        return kStorageMode;
    }

    public boolean isReady(){
        return kIsAtState;
    }

    public double getIntakeSpeed(){
        return kIntakeMotor.get();
    }

    public void moveIntakeSlider(double speed){
        kIntakeSlider.set(speed);
    }

    public void stopIntakeSlider(){
        kIntakeSlider.set(0);
    }

    public void disablePID(){
        kPIDEnabled = false;
    }

    public void enablePID(){
        kPIDEnabled = true;
    }

    @Override
    public void periodic(){
        if (kPIDEnabled)
        {
            kIsAtState = kSliderPID.atSetpoint();
            if (Robot.isReal()){
                PIDOutput = MathUtil.clamp(kSliderPID.calculate(kIntakeSlider.getPosition().getValueAsDouble()*Constants.IntakeSubsystemConstants.kEncoderToInchesMult,kSliderTarget), -IntakeSubsystemConstants.kSliderMaxSpeedOutput, IntakeSubsystemConstants.kSliderMaxSpeedOutput);
            } else {
                PIDOutput = MathUtil.clamp(kSliderPID.calculate(kIntakeSim.getPositionMeters()*IntakeSubsystemConstants.kSimLenBaseMult,kSliderTarget), -IntakeSubsystemConstants.kSimMaxSpeed, IntakeSubsystemConstants.kSimMaxSpeed);
            }
            kIntakeSlider.set(PIDOutput);
        }
        
        SmartDashboard.putNumber("Subsystems/IntakeSubsystem/Intake Rack PID Output: ", PIDOutput);
        SmartDashboard.putString("Subsystems/IntakeSubsystem/Current Action Mode: ", kActionMode.name());
        SmartDashboard.putString("Subsystems/IntakeSubsystem/Current Storage Mode: ", kStorageMode.name());
        SmartDashboard.putNumber("Subsystems/IntakeSubsystem/Intake Rack Encoder Position: ", kIntakeSlider.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Subsystems/IntakeSubsystem/Intake Rack Stator Current: ", kIntakeSlider.getStatorCurrent().getValueAsDouble());
    } 

    @Override
    public void simulationPeriodic(){
        kIntakeSim.setInput(PIDOutput * IntakeSubsystemConstants.kSimMultiplier);

        kIntakeSim.update(IntakeSubsystemConstants.kSimdt);

        kSimDisp.setLength(kIntakeSim.getPositionMeters()*Constants.IntakeSubsystemConstants.kSimLenMult);
        SmartDashboard.putNumber("Intake Length Horizontal", kIntakeSim.getPositionMeters()*39.3701*Math.cos(Math.toRadians(15)));
    }
}
