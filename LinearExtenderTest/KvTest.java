package org.firstinspires.ftc.teamcode.LinearExtenderTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import org.firstinspires.ftc.teamcode.NonOpModes.MotionProfile;
import org.firstinspires.ftc.teamcode.NonOpModes.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "KvTest", group = "Extender")
public class KvTest extends LinearOpMode {
  
  private RobotController r;
  MotionProfile mp;
  int index;
  double position;
  
  public enum ExtenderState{
    Searching,
    Grabbing,
    Lifting,
    Placing,
    AwayDriving,
    UnderBridge;
    private static ExtenderState[] vals = values();
    
    public ExtenderState next() {
        return vals[(this.ordinal()+1) % vals.length];
    }
    
    public ExtenderState prev() {
        return vals[(this.ordinal()-1) % vals.length];
    }
  }
  
  public void initState(ExtenderState state) {
    switch(state) {
      case Searching:
        mp = new MotionProfile(position, 2, 18, 40);
        r.GripBlock.setPosition(1);
        // code block
        break;
      case Grabbing:
        mp = new MotionProfile(position, 0, 18, 40);
        // code block
        break;
      case Lifting:
        mp = new MotionProfile(position, index*10+5, 18, 40);
        r.GripBlock.setPosition(0);
        break;
      case Placing:
        mp = new MotionProfile(position, index*10+2, 18, 40);
        r.GripBlock.setPosition(0);
        break;
      case AwayDriving:
        mp = new MotionProfile(position, index*10+5, 18, 40);
        r.GripBlock.setPosition(1);
        break;
      case UnderBridge:
        mp = new MotionProfile(position, 0, 18, 40);
        r.GripBlock.setPosition(0);
        index++;
        break;
      default:
        // code block
      }
    }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {




    // Support class for robot control
    r = new RobotController(this);

    
    
    
    
    // Reverse one of the drive motors. 
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    r.InitAndWait();
    double prevposition = 0;
    boolean prevA = false, prevB = false, prevUp = false, prevDown = false;
    
    
    // This is the height of the lift for lifting;
    index = 0;
    // This variable tracks which state the extender is in;
    ExtenderState state = ExtenderState.Searching;
    boolean newState = false;
    
    r.resetExtender();

    double position = r.getExtenderPosition();
    
    PIDFController pidf = new PIDFController(0,0,0);//0.07,0.04,0.004);
    pidf.enable();
    pidf.reset();
    double newTime = r.runtime.time();
    initState(state);
    
    double targetangle = 0;
    
    while (opModeIsActive()) {
      r.BasicLoopTele();
      position = r.getExtenderPosition();
      double t = r.runtime.time() - newTime;
      double power = pidf.performPIDF(position, mp.getPos(t), mp.getV(t), mp.getA(t));
      boolean a = gamepad1.a, b = gamepad1.b, up = gamepad1.dpad_up, down = gamepad1.dpad_down;
      if (state == ExtenderState.Grabbing &&  t > mp.tTotal+0.5) {
        r.GripBlock.setPosition(0);
      }
      newState = false;
      if (a && !prevA) {
        state = state.next();
        newState = true;
      }
      if (b && !prevB) {
        state = state.prev();
        newState = true;
      }
      if (newState) {
        initState(state);
        pidf.reset();
        newTime = r.runtime.time();
      }
      telemetry.addData("state", state);

      
      // Adds or subtracts one from the stone index height
      if (up&&!prevUp) {
        if (index<5) {
          index++;
        }
      }
      else if(down&&!prevDown) {
        if (index>0) {
          index--;
        }
      }
      prevDown = down;
      prevUp = up;
      prevA = a;
      prevB = b;
      
      r.DriveGyro();
      telemetry.addData("position", position) ;
      telemetry.addData("index", index) ;
      telemetry.addData("t", t);
      telemetry.addData("speed", (position-prevposition)/r.dt);
      telemetry.addData("power", power);
      telemetry.addData("error", pidf.getError());
      r.extender.setPower(power);
      telemetry.update();
      prevposition = position;
      sleep(50);
      pidf.setPID(0.04,0.01,0.01);
    }
  }
}
