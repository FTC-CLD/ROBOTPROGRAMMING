/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Unused;

import android.app.Activity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import android.graphics.Color;
import android.view.View;
import android.widget.TextView;
import android.widget.RelativeLayout;
import     android.util.TypedValue;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/*
 * This is an example LinearOpMode that shows how to use a color sensor in a generic
 * way, insensitive which particular make or model of color sensor is used. The opmode
 * assumes that the color sensor is configured with a name of "sensor_color".
 *
 * If the color sensor has a light which is controllable, you can use the X button on
 * the gamepad to toggle the light on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Scherm testererere")

public class TestAndroidGraphics extends LinearOpMode {
  public TextView mTextView;
  public int t;
  /** The colorSensor field will contain a reference to our color sensor hardware object */
  NormalizedColorSensor colorSensor;
  /** The relativeLayout field is used to aid in providing interesting visual feedback
   * in this sample application; you probably *don't* need something analogous when you
   * use a color sensor on your robot */
  RelativeLayout relativeLayout;

  /**
   * The runOpMode() method is the root of this LinearOpMode, as it is in all linear opModes.
   * Our implementation here, though is a bit unusual: we've decided to put all the actual work
   * in the main() method rather than directly in runOpMode() itself. The reason we do that is that
   * in this sample we're changing the background color of the robot controller screen as the
   * opmode runs, and we want to be able to *guarantee* that we restore it to something reasonable
   * and palatable when the opMode ends. The simplest way to do that is to use a try...finally
   * block around the main, core logic, and an easy way to make that all clear was to separate
   * the former from the latter in separate methods.
   */
  @Override public void runOpMode() throws InterruptedException {

    // Get a reference to the RelativeLayout so we can later change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    waitForStart();
    
    telemetry.addData("werkt", "");
    telemetry.update();
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = (RelativeLayout)((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    telemetry.addData("Werkt","nog steeds");
    mTextView = new TextView(hardwareMap.appContext);
  relativeLayout.post(new Runnable() {
        public void run() {
        mTextView.setText("1");
        mTextView.setTextSize(TypedValue.COMPLEX_UNIT_SP,500);
        RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.WRAP_CONTENT, RelativeLayout.LayoutParams.WRAP_CONTENT);
        params.addRule(RelativeLayout.CENTER_IN_PARENT, RelativeLayout.TRUE);
  
          relativeLayout.addView(mTextView, params);
        }
    }
  );
    t = 0;
    while (opModeIsActive()) {
      t++;
      telemetry.update();
      relativeLayout.post(new Runnable() {
        public void run() {
           mTextView.setText(String.valueOf(t));
        }
      });
     
      idle();
      sleep(1000);
    }
    relativeLayout.removeView(mTextView);
  }

}
