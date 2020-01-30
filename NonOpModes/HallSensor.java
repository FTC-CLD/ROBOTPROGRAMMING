package org.firstinspires.ftc.teamcode.NonOpModes;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

/*
 * Created by Dryw Wade
 *
 * Driver for Adafruit's MCP9808 temperature sensor
 *
 * This version of the driver does not make use of the I2C device with parameters. This means the
 * settings for the configuration register are hard coded and cannot be changed by the user, nor can
 * they be different for each OpMode.
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out
@I2cSensor(name = "HallSensor", description = "HallSensor for encoders", xmlTag = "Hall")
public class HallSensor extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////
    private int prevAngleRaw,absAngleRaw;
    
    public double getAngle()
    {
    return -getAngleRaw()/4096.0*2*Math.PI;
    }

    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Raw Register Reads
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public int getAngleRaw()
    {
        return prevAngleRaw;
    }
    
    public void updateAngleRaw()
    {
        byte[] hallInput = readByteArr(Register.ANG);
        int a = hallInput[0]&0x0F;
        int b = hallInput[1]&0xFF;
        int curAngle = a*256+b;
        int difference = prevAngleRaw-curAngle-absAngleRaw;
        if (Math.abs(difference) > 2000) {
            absAngleRaw += Math.signum(difference)*4096;
        }
        prevAngleRaw = absAngleRaw+curAngle;
    }

    public int getField()
    {
        byte[] hallInput = readByteArr(Register.FIELD);
        int a = hallInput[0]&0x0F;
        int b = hallInput[1]&0xFF;
        return a*256+b;
    }

    public int getErr()
    {
        byte[] hallInput = readByteArr(Register.ERR);
        int a = hallInput[0]&0x0F;
        int b = hallInput[1]&0xFF;
        return a*256+b;
    }
    
    public String getErrStr()
    {
        int err=getErr();
        String errcodes = "XEXOIECRNRATAHALOVUVMHML";
        String outp="";
        int bit=1<<11;
        for (int i=0; i<12; i++){
            if ((err&bit)==bit) {
                outp+=errcodes.substring(i*2,i*2+2)+" ";
            }
            bit>>=1;
        }
        if (outp=="") { outp="No errors"; }
        return outp;
        
    }
    
    public void clrErr()
    { 
        byte[] buf={0x01,0x46};
        writeByteArr(Register.CTRL,buf,2);
    }
    
   
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////


    protected byte[] readByteArr(Register reg)
    {
        return deviceClient.read(reg.bVal, 2);
        
    }

    protected void writeByteArr(Register reg, byte[] buf,int l)
    {  
       deviceClient.write(reg.bVal,buf);
       
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public enum Register
    {
        FIRST(0),
        CONFIGURATION(0x01),
        CTRL(0x1E),
        ANG(0x20),
        FIELD(0x2A),
        ERR(0x24),
        MANUFACTURER_ID(0x06);
        
        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }



    public enum AlertControl
    {
        ALERT_DISABLE(0x0000),
        ALERT_ENABLE(0x0008);

        public int bVal;

        AlertControl(int bVal)
        {
            this.bVal = bVal;
        }
    }

    // More settings are available on the sensor, but not included here. Could be added later

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x0c);

    public HallSensor(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
       
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode

    }

    @Override
    protected synchronized boolean doInitialize()
    {

        prevAngleRaw=0;
        absAngleRaw=0;

        // Mask out alert signal bit, which we can't control
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName()
    {
        return "Allegro A1335 Precision Hall-Effect Angle Sensor";
    }
}
