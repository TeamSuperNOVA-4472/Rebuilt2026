package frc.robot.util;

/**
 * Utility class to organize the use of PID constants
 */
public class PIDConstants {
    private final double mP;
    private final double mI;
    private final double mD;
    private final double mF;

    /**
     * Constructor
     * @param pP The Proportional constant.
     * @param pI The Integral constant.
     * @param pD The Differential constant.
     */
    public PIDConstants(double pP, double pI, double pD) {
        this(pP, pI, pD, 0);
    }

    /**
     * Constructor
     * @param pP The Proportional constant.
     * @param pI The Integral constant.
     * @param pD The Differential constant.
     * @param pF The Feed-Forward constant.
     */
    public PIDConstants(double pP, double pI, double pD, double pF) {
        mP = pP;
        mI = pI;
        mD = pD;
        mF = pF;
    }

    /**
     * @return The Proportional constant
     */
    public double getP() {
        return mP;
    }

    /**
     * @return The Integral constant
     */
    public double getI() {
        return mI;
    }

    /**
     * @return The Derivative constant
     */
    public double getD() {
        return mD;
    }

    /**
     * @return The Feed-Forward constant
     */
    public double getF() {
        return mF;
    }
}
