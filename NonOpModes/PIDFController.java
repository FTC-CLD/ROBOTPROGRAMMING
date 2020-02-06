package org.firstinspires.ftc.teamcode.NonOpModes;
import com.qualcomm.robotcore.util.ElapsedTime;
// PID controller courtesy of Peter Tischler, with modifications.

public class PIDFController
{
    private double m_P;                     // factor for "proportional" control
    private double m_I;                     // factor for "integral" control
    private double m_D;                     // factor for "derivative" control
    private double m_input;                 // sensor input for pid controller
    private double m_maximumOutput = 1.0;    // |maximum output|
    private double m_minimumOutput = -1.0;    // |minimum output|
    private boolean m_continuous = false;    // do the endpoints wrap around? eg. Absolute encoder
    private boolean m_enabled = false;      // is the pid controller enabled
    private double m_prevError = 0.0;
    private double prevInput = 0.0;// the prior sensor input (used to compute velocity)
    private double m_totalError = 0.0;      // the sum of the errors for use in the integral calc
    private double m_tolerance = 0.05;      // the percentage error that is considered on target
    private double m_setpoint = 0.0;
    private double m_error = 0.0;
    private double m_result = 0.0;
    private double prevt = 0.0, a, v, pos;
    final public static double kG = 0.092, kF = 0.108, posToY = 57.0/820.0, kV = 0.01, kA = 0.0001;
    private ElapsedTime runtime;

    /**
     * Allocate a PID object with the given constants for P, I, D
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient 
     */
    public PIDFController(double Kp, double Ki, double Kd)
    {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }


    public double getK(double position,double velocity) {
        if (velocity>0) {
           return position*(0.26-0.190)/54+0.34;
        }
        else if (velocity ==0){
           return 0.01; 
        }
        else {
            if (velocity > -2) {
                return -0.03+position*0.10/50; 
            }
            return position*0.10/50; 
        }
    }
    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should only be called by the PIDTask
     * and is created during initialization.
     */
    private void calculate()
    {
        int     sign = 1;

        // If enabled then proceed into controller calculations
        if (m_enabled)
        {
            // Calculate the error signal
            m_error = m_setpoint - m_input;

            double dt = runtime.time()-prevt;
            if (dt==0) dt = 0.01;

            // Integrate the errors as long as the upcoming integrator does
            // not exceed the minimum and maximum output thresholds.

            if ((Math.abs(m_totalError + m_error*dt) * m_I < m_maximumOutput) &&
                    (Math.abs(m_totalError + m_error*dt) * m_I > m_minimumOutput))
                m_totalError += m_error*dt;

            // Perform the primary PID calculation
            // The D term is changed to account for the speed at which the system has to be moving
            m_result = m_P * m_error + m_I * m_totalError + m_D * (v-(m_input - prevInput)/dt) + v * kV + a*kA + getK(m_input, v);

            // Set the current error to the previous error for the next cycle.
            m_prevError = m_error;
            prevInput = m_input;

            if (m_result < 0) sign = -1;    // Record sign of result.

            // Make sure the final result is within bounds. If we constrain the result, we make
            // sure the sign of the constrained result matches the original result sign.
            if (Math.abs(m_result) > m_maximumOutput)
                m_result = m_maximumOutput * sign;
            else if (Math.abs(m_result) < m_minimumOutput)
                m_result = m_minimumOutput * sign;
            prevt = runtime.time();
        }
        
    }
    
    public double getError() {
        return m_error;
    }
    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public void setPID(double p, double i, double d)
    {
        m_P = p;
        m_I = i;
        m_D = d;
    }

    

    /**
     * Return the current PID result for the specified input.
     * @param input The input value to be used to calculate the PID result.
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    public double performPIDF(double input, double setpoint, double targetV, double targetA)
    {
        m_setpoint = setpoint;
        v = targetV;
        a = targetA;
        m_input = input;
        calculate();
        return m_result;
    }





    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output, always positive
     * @param maximumOutput the maximum value to write to the output, always positive
     */
    public void setOutputRange(double minimumOutput, double maximumOutput)
    {
        m_minimumOutput = Math.abs(minimumOutput);
        m_maximumOutput = Math.abs(maximumOutput);
    }
    
    /**
     * Begin running the PIDController
     */
    public void enable() 
    {
        m_enabled = true;
        runtime = new ElapsedTime();
        prevt = 0;
    }

    /**
     * Stop running the PIDController.
     */
    public void disable() 
    {
        m_enabled = false;
    }

    /**
     * Reset the previous error,, the integral term
     */
    public void reset()
    {
        m_prevError = 0;
        m_totalError = 0;
        m_result = 0;
    }


}