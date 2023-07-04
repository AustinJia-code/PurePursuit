package org.firstinspires.ftc.teamcode.PurePursuit;
//modified from FTCLib PIDF controller

public class PID {
    private double kP, kI, kD;
    private double setPoint;
    private double measuredValue;
    private double minIntegral, maxIntegral;
    private double errorVal_p;
    private double errorVal_v;
    private double totalError;
    private double prevErrorVal;
    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;
    private double lastTimeStamp;
    private double period;
    private double output;

    public PID(double kp, double ki, double kd) {
        this(kp, ki, kd, 0, 0);
    }
    public PID(double kp, double ki, double kd, double sp, double pv) {
        kP = kp;
        kI = ki;
        kD = kd;

        setPoint = sp;
        measuredValue = pv;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        lastTimeStamp = 0;
        period = 0;

        errorVal_p = setPoint - measuredValue;
        reset();
    }
    public void reset() {
        totalError = 0;
        prevErrorVal = 0;
        lastTimeStamp = 0;
    }
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = positionTolerance;
        errorTolerance_v = velocityTolerance;
    }
    public double getSetPoint() {
        return setPoint;
    }
    public void setSetPoint(double sp) {
        setPoint = sp;
        errorVal_p = setPoint - measuredValue;
        errorVal_v = (errorVal_p - prevErrorVal) / period;
    }
    public boolean atSetPoint() {
        return Math.abs(errorVal_p) < errorTolerance_p
                && Math.abs(errorVal_v) < errorTolerance_v;
    }
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD};
    }
    public double getPositionError() {
        return errorVal_p;
    }
    public double[] getTolerance() {
        return new double[]{errorTolerance_p, errorTolerance_v};
    }
    public double getVelocityError() {
        return errorVal_v;
    }
    public double calculate() {
        return calculate(measuredValue);
    }
    public double calculate(double pv, double sp) {
        // set the setpoint to the provided value
        setSetPoint(sp);
        return calculate(pv);
    }
    public double calculate(double pv) {
        prevErrorVal = errorVal_p;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        if (measuredValue == pv) {
            errorVal_p = setPoint - measuredValue;
        } else {
            errorVal_p = setPoint - pv;
            measuredValue = pv;
        }

        if (Math.abs(period) > 1E-6) {
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }

        totalError += period * (setPoint - measuredValue);
        totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);

        // returns u(t)
        output = kP * errorVal_p + kI * totalError + kD * errorVal_v * setPoint;

        return output;
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
    }

    public void clearTotalError() {
        totalError = 0;
    }

    public void setP(double kp) {
        kP = kp;
    }

    public void setI(double ki) {
        kI = ki;
    }

    public void setD(double kd) {
        kD = kd;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getPeriod() {
        return period;
    }

    public double getOutput(){
        return output;
    }
}