package org.firstinspires.ftc.teamcode.NonOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import org.firstinspires.ftc.teamcode.NonOpModes.MotionProfile;
import org.firstinspires.ftc.teamcode.NonOpModes.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Extender {
  final double Grabbing = 0.5;
  private RobotController r;
  MotionProfile mp;
  PIDFController pidf;
  ExtenderState state;
  double newTime;
  public int index;
  double t;
  double position, error;
  double prevposition;
  
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
        return vals[(this.ordinal()-1+vals.length) % vals.length];
    }
  }
  
    public void initState(ExtenderState state, double position) {
    switch(state) {
      case Searching:
        mp = new MotionProfile(position, 6, 19, 40);
        r.GripBlock.setPosition(1);
        // code block
        break;
      case Grabbing:
        mp = new MotionProfile(position, -0.15, 15, 40);
        // code block
        break;
      case Lifting:
        mp = new MotionProfile(position, index*10.16+7, 12, 130);
        r.GripBlock.setPosition(Grabbing);
        break;
      case Placing:
        mp = new MotionProfile(position, index*10.16+3, 10, 40);
        r.GripBlock.setPosition(Grabbing);
        break;
      case AwayDriving:
        mp = new MotionProfile(position, index*10.16+7, 15, 40);
        r.GripBlock.setPosition(1);
        break;
      case UnderBridge:
        mp = new MotionProfile(position, -0.3, 15, 50);
        r.GripBlock.setPosition(Grabbing);
        if (index<5) index++;
        break;
      default:
        // code block
      }
    }

    public Extender(RobotController R) {
      r = R;
      prevposition = 0;
      
      
       // This is the height of the lift for lifting;
      index = 0;
      // This variable tracks which state the extender is in;
      state = ExtenderState.Searching;
      position = r.getExtenderPosition();
      
      pidf = new PIDFController(0,0,0);//0.07,0.04,0.004);
      pidf.enable();
      pidf.reset();
      newTime = r.runtime.time(); 
      t = 0; 
      initState(state, position);

    }

    public void enterState(ExtenderState state) {
      initState(state, position);
      pidf.reset();
      newTime = r.runtime.time();
      t = 0; 
    }
    
    public void raise() {
      if (index<5) index++;
      enterSame();
    }
    public void lower() {
      if (index>0) index--;
      enterSame();
    }
    
    public boolean running() {
      if (state == ExtenderState.Grabbing) {
        return t < mp.tTotal+1;
      }
      else {
        return t < mp.tTotal+0.3;
      }
    }
    public void enterNext() {
      state = state.next();
      enterState(state);
    }
    public void enterPrev() {
      state = state.prev();
      enterState(state);
    }
    
    public void enterSame() {
      enterState(state);
    }
    public void update() {
      position = r.getExtenderPosition();
      t = r.runtime.time() - newTime;
      double power = pidf.performPIDF(position, mp.getPos(t), mp.getV(t), mp.getA(t));
      r.extender.setPower(power);
      if (state == ExtenderState.Grabbing &&  t > mp.tTotal+0.06) {
        r.GripBlock.setPosition(Grabbing); 
        
      }
      prevposition = position;
      pidf.setPID(0.04,0.03,0.03);
  }
}
