package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX kClimbMotor;
    private ClimbState kState = ClimbState.STORED;
    private PIDController kClimbController;

    
    private final DCMotor kClimbSimMotor;
    private final ElevatorSim kClimbSim;
    private final Mechanism2d kSimSpace;
    private final MechanismRoot2d kSimRoot;
    private final MechanismLigament2d kSimDisp;


    public enum ClimbState{
        STORED,
        UP,
        CLIMB
    }
    private boolean kAtSetpoint = false;
    private double kTarget = 0;
    private double PIDOutput = 0;

    public ClimbSubsystem()
    {
        kClimbMotor = new TalonFX(ClimbConstants.kClimbMotorPort, ClimbConstants.kClimbCanbus);

        TalonFXConfiguration kClimbConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs kClimbCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs kClimbMotorConfig = new MotorOutputConfigs();
        kClimbController = new PIDController(2, 0, 0);
        kClimbMotor.getConfigurator().refresh(kClimbConfig);
        kClimbMotor.getConfigurator().refresh(kClimbCurrentConfig);
        kClimbMotor.getConfigurator().refresh(kClimbMotorConfig);
        kClimbCurrentConfig.SupplyCurrentLimit = ClimbConstants.kClimbSupplyLimit;
        kClimbCurrentConfig.SupplyCurrentLimitEnable = ClimbConstants.kClimbSupplyLimitEnabled;
        kClimbCurrentConfig.StatorCurrentLimitEnable = ClimbConstants.kClimbStatorLimitEnabled;
        kClimbCurrentConfig.StatorCurrentLimit = ClimbConstants.kClimbStatorLimit;
        kClimbCurrentConfig.SupplyCurrentLowerLimit = ClimbConstants.kClimbSupplyLimit;
        kClimbMotorConfig.NeutralMode = ClimbConstants.kClimbNeutralMode;
        kClimbConfig.withCurrentLimits(kClimbCurrentConfig);
        kClimbConfig.withMotorOutput(kClimbMotorConfig);
        kClimbMotor.getConfigurator().apply(kClimbConfig);

        kClimbSimMotor = DCMotor.getKrakenX60(ClimbConstants.kSimNumMotors);
        kClimbSim = new ElevatorSim(kClimbSimMotor, Constants.ClimbConstants.kGearing, Constants.ClimbConstants.kMass, Constants.ClimbConstants.kDrumRadius, 0, Constants.ClimbConstants.kMaxLen, true, 0, 0, 0);
        kSimSpace = new Mechanism2d(ClimbConstants.kSimWidth, ClimbConstants.kSimHeight);
        kSimRoot = kSimSpace.getRoot(ClimbConstants.kSimRootName, ClimbConstants.kSimX, ClimbConstants.kSimY);
        kSimDisp = kSimRoot.append(new MechanismLigament2d("Climb", kClimbSim.getPositionMeters()*Constants.ClimbConstants.kSimLenMult,90));
        SmartDashboard.putData("Climbsim", kSimSpace);
    }

    public void setVoltage(double volts)
    {
        kClimbMotor.setVoltage(volts);
    }

    public void setState(ClimbState mState){
        kState = mState;
        moveWithState();
    }
    @Override
    public void periodic(){
        /*kAtSetpoint = kClimbController.atSetpoint();
        if (kState == ClimbState.CLIMB && !kAtSetpoint && kClimbMotor.getPosition().getValueAsDouble() * ClimbConstants.kEncoderToInches > ClimbConstants.kClimb){
            if (Robot.isReal()){
                PIDOutput = MathUtil.clamp(kClimbController.calculate(kClimbMotor.getPosition().getValueAsDouble() * ClimbConstants.kEncoderToInches) - ClimbConstants.kClimbVoltage,-10, 10);
            } else{
                PIDOutput = MathUtil.clamp(kClimbController.calculate(kClimbSim.getPositionMeters() * ClimbConstants.kSimLenBaseMult) - ClimbConstants.kClimbVoltage,-10, 10);
            }
        }
        else if(!kAtSetpoint){
            if (Robot.isReal()){
                PIDOutput = MathUtil.clamp(kClimbController.calculate(kClimbMotor.getPosition().getValueAsDouble() * ClimbConstants.kEncoderToInches), -10, 10);
            } else{
                PIDOutput = MathUtil.clamp(kClimbController.calculate(kClimbSim.getPositionMeters() * ClimbConstants.kSimLenBaseMult), -10, 10);
            }
        }
        else{
            PIDOutput = 0;
        }
        setVoltage(PIDOutput);*/
        SmartDashboard.putNumber("Subsystems/ClimbSubsystem/Encoder Position: ", kClimbMotor.getPosition().getValueAsDouble());
    }
    private void moveWithState(){
        switch (kState) {
            case STORED:
                kTarget = ClimbConstants.kStored;
                break;
        
            case UP:
                kTarget = ClimbConstants.kUP;
                break;

            case CLIMB:
                kTarget = ClimbConstants.kClimb;
                break;
        }
        kClimbController.setSetpoint(kTarget);
    }
    public ClimbState getClimbState(){
        return kState;
    }
    public boolean isAtSetpoint(){
        return kAtSetpoint;
    }
    @Override
    public void simulationPeriodic(){
        kClimbSim.setInput(PIDOutput);

        kClimbSim.update(ClimbConstants.kSimdt);

        kSimDisp.setLength(kClimbSim.getPositionMeters()*Constants.ClimbConstants.kSimLenMult);
        SmartDashboard.putNumber("Climb Height", kClimbSim.getPositionMeters() * ClimbConstants.kSimLenBaseMult);
        SmartDashboard.putNumber("Climb Target", kTarget);
    }
}