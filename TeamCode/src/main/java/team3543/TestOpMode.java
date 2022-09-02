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

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test OpMode", group="Test")
//@Disabled
public class TestOpmode extends LinearOpMode {

   // Declare OpMode members.
   private ElapsedTime runtime = new ElapsedTime();

   @Override
   public void runOpMode()
   {
      double lfEncTime = -1.0;
      double rfEncTime = -1.0;
      double lbEncTime = -1.0;
      double rbEncTime = -1.0;

      DcMotor lfDrive  = hardwareMap.get(DcMotor.class, "lfWheel");
      DcMotor rfDrive  = hardwareMap.get(DcMotor.class, "rfWheel");
      DcMotor lbDrive  = hardwareMap.get(DcMotor.class, "lbWheel");
      DcMotor rbDrive  = hardwareMap.get(DcMotor.class, "rbWheel");

      lfDrive.setDirection(DcMotor.Direction.REVERSE);
      lbDrive.setDirection(DcMotor.Direction.REVERSE);

      // Wait for the game to start (driver presses PLAY)
      waitForStart();
      runtime.reset();

      int lfEnc = lfDrive.getCurrentPosition();
      int rfEnc = rfDrive.getCurrentPosition();
      int lbEnc = lbDrive.getCurrentPosition();
      int rbEnc = rbDrive.getCurrentPosition();

      telemetry.addData("Start Encoders", "[%.3f]: lfEnc=%.0f, rfEnc=%.0f, lbEnc=%.0f, rbEnc=%.0f",
                        runtime.seconds(), lfEnc, rfEnc, lbEnc, rbEnc);
      telemetry.update();

      lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      // run until the end of the match (driver presses STOP)
      while (opModeIsActive())
      {
         double currTime = runtime.seconds();

         if (lfEncTime < 0.0 && lfEnc == 0)
         {
            lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lfEncTime = currTime;
         }

         if (rfEncTime < 0.0 && rfEnc == 0)
         {
            rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rfEncTime = currTime;
         }

         if (lbEncTime < 0.0 && lbEnc == 0)
         {
            lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lbEncTime = currTime;
         }

         if (rbEncTime < 0.0 && rbEnc == 0)
         {
            rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rbEncTime = currTime;
         }

         telemetry.addData("Current Time", "time=%.3f", currTime);
         telemetry.addData("Front Encoders", "lfEnc[%.3f]=%.0f, rfEnc[%.3f]=%.0f", lfEnc, lfEncTime, rfEnc, rfEncTime);
         telemetry.addData(" Back Encoders", "lbEnc[%.3f]=%.0f, rbEnc[%.3f]=%.0f", lbEnc, lbEncTime, rbEnc, rbEncTime);
         telemetry.update();
      }
   }
}
