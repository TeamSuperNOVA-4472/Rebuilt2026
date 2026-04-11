package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerSubsystem extends SubsystemBase {
      private final PowerDistribution mPdh = new PowerDistribution(1, ModuleType.kRev);

      /*@Override
      public void periodic() {
        //SmartDashboard.putData("PDH", mPdh);
      }*/
}
