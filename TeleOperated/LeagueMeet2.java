package org.firstinspires.ftc.teamcode.TeleOperated;

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

@TeleOp(name = "LeagueMeet2")
public class LeagueMeet2 extends LinearOpMode {
  
  private RobotController r;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    // Support class for robot control
    r = new RobotController(this);

    r.InitAndWait();
    double prevposition = 0;
    boolean prevA = false, prevB = false, prevX = false, prevY = false, prevUp = false, prevDown = false;
    
    boolean hooksDown = false, capstoneDown = false;
    r.Foundation1.setPosition(0);
    r.Foundation2.setPosition(1);
    
    while (opModeIsActive()) {
      r.BasicLoopTele();
      boolean a = gamepad1.a, b = gamepad1.b, up = gamepad1.dpad_up, down = gamepad1.dpad_down, x = gamepad1.x, y = gamepad1.y;

      if (a && !prevA) {
        r.manageExtender.enterNext();
      }
      else if (b && !prevB) {
        r.manageExtender.enterPrev();
      }

      // Adds or subtracts one from the stone index height
      if (up&&!prevUp) {
        r.manageExtender.raise();
      }
      else if(down&&!prevDown) {
        r.manageExtender.lower();
      }
      
      if (x && !prevX) {
        hooksDown = !hooksDown;
        if (hooksDown) {
          r.Foundation1.setPosition(1);
          r.Foundation2.setPosition(0);
        } else {
          r.Foundation1.setPosition(0);
          r.Foundation2.setPosition(1);
        }

      }
      if (y && !prevY) {
        capstoneDown = !capstoneDown;
        if (capstoneDown) {
          r.Capstone.setPosition(1);

        } else {
          r.Capstone.setPosition(0);
        }

      }
      prevDown = down;
      prevUp = up;
      prevA = a;
      prevB = b;
      prevX = x;
      prevY = y;
      
      r.DriveGyro();
      idle();
    }
  }
}
