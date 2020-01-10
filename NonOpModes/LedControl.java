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
@I2cSensor(name = "LedControl", description = "Led Controller via I2c", xmlTag = "LedControl")
public class LedControl extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////
   
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Raw Register Reads
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public void setLed(int lednr, int r, int g, int b) {
        byte[] i2cbuf;
        i2cbuf = new byte[4];
        i2cbuf[0]=(byte)lednr;
        i2cbuf[1]=(byte)r;
        i2cbuf[2]=(byte)g;
        i2cbuf[3]=(byte)b;
        writeByteArr(Register.RGBLED,i2cbuf,4);
    }

    
    public void setCompass(int heading) {
        if (heading<0) {heading+=360;}
        int hibyte = heading>>8;
        byte[] i2cbuf;
        i2cbuf = new byte[2];
        i2cbuf[0]=(byte)(hibyte&0xff);
        i2cbuf[1]=(byte)(heading&0xff);
        writeByteArr(Register.COMPASS,i2cbuf,2);
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
        RGBLED(0x01),
        COMPASS(0x02);
        
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

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x05);

    public LedControl(I2cDeviceSynch deviceClient)
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
