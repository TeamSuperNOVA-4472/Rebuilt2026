package frc.robot;

import java.lang.reflect.Field;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.GameConstants;

public class GameDataHelpers {
    public enum Situation {
        INACTIVE(false),
        ACTIVE(true),
        AUTONOMOUS(true),
        PRACTICE(true);

        private final boolean mHubActive;

        Situation(boolean pHubActive)
        {
            mHubActive = pHubActive;
        }

        public boolean get() { return mHubActive; }
    }

    public static Situation getSituation()
    {
        if (!DriverStation.isFMSAttached()) { return Situation.PRACTICE; }
        if (DriverStation.isAutonomous()) { return Situation.AUTONOMOUS; }
        
        double matchTime = DriverStation.getMatchTime();

        if (matchTime > GameConstants.kShiftChanges[0])
        {
            return Situation.ACTIVE;
        }
        else if (matchTime > GameConstants.kShiftChanges[1])
        {
            return hasFirstShift() ? Situation.ACTIVE : Situation.INACTIVE;
        }
        else if (matchTime > GameConstants.kShiftChanges[2])
        {
            return hasFirstShift() ? Situation.INACTIVE : Situation.ACTIVE;
        }
        else if (matchTime > GameConstants.kShiftChanges[3])
        {
            return hasFirstShift() ? Situation.ACTIVE : Situation.INACTIVE;
        }
        else if (matchTime > GameConstants.kShiftChanges[4])
        {
            return hasFirstShift() ? Situation.INACTIVE : Situation.ACTIVE;
        }
        else
        {
            return Situation.ACTIVE;
        }
    }

    public static boolean isHubActive() { return getSituation().get(); }

    public static boolean hasFirstShift()
    {
        String gameMessage = DriverStation.getGameSpecificMessage();
        if (gameMessage.isEmpty()) return true;

        boolean isRed = FieldMathHelpers.isRedAlliance();
        boolean didRedWinAuto;

        switch(gameMessage.charAt(0))
        {
            case 'R' -> didRedWinAuto = true;
            case 'B' -> didRedWinAuto = false;
            default -> { return true; }
        }

        // Negated XOR returns true if both are true or both are false
        return !(didRedWinAuto^isRed) ? false : true;
    }
}
