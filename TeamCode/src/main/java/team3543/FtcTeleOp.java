/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team3543;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;

/**
 * This class contains the TeleOp Mode program.
 */
@TeleOp(name="FtcTeleOp", group="Ftc3543")
public class FtcTeleOp extends FtcOpMode
{
    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    private boolean invertedDrive = false;
    private double drivePowerScale = 1.0;
    private double armPowerScale = 1.0;
    private boolean armManualOverride = false;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void initRobot()
    {
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Create and initialize Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this::driverButtonEvent);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this::operatorButtonEvent);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);
        printPerformanceMetrics(robot.globalTracer);
    }   //stopMode

    /**
     * This method is called periodically at a slow rate. Typically, you put code that doesn't require frequent
     * update here. For example, TeleOp joystick code or status display code can be put here since human responses
     * are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void slowPeriodic(double elapsedTime)
    {
        //
        // DriveBase subsystem.
        //
        if (robot.robotDrive != null)
        {
            switch (RobotParams.ROBOT_DRIVE_MODE)
            {
                case TANK_MODE:
                {
                    double leftPower = driverGamepad.getLeftStickY(true)*drivePowerScale;
                    double rightPower = driverGamepad.getRightStickY(true)*drivePowerScale;
                    robot.robotDrive.driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                    robot.dashboard.displayPrintf(1, "Tank:left=%.1f,right=%.1f,inv=%s",
                                                  leftPower, rightPower, invertedDrive);
                    break;
                }

                case HOLONOMIC_MODE:
                {
                    double x = driverGamepad.getLeftStickX(true)*drivePowerScale;
                    double y = driverGamepad.getRightStickY(true)*drivePowerScale;
                    double rot = (driverGamepad.getRightTrigger(true) - driverGamepad.getLeftTrigger(true))*
                                 drivePowerScale;
                    robot.robotDrive.driveBase.holonomicDrive(x, y, rot, invertedDrive);
                    robot.dashboard.displayPrintf(1, "Holonomic:x=%.1f,y=%.1f,rot=%.1f,inv=%s",
                                                  x, y, rot, invertedDrive);
                    break;
                }

                case TIM_MODE:
                {
                    double x = driverGamepad.getRightStickX(true)*drivePowerScale;
                    double y = driverGamepad.getRightStickY(true)*drivePowerScale;
                    double rot = driverGamepad.getLeftStickX(true)*drivePowerScale;
                    robot.robotDrive.driveBase.holonomicDrive(x, y, rot, invertedDrive);
                    robot.dashboard.displayPrintf(1, "Tim:x=%.1f,y=%.1f,rot=%.1f,inv=%s",
                                                  x, y, rot, invertedDrive);
                    break;
                }
            }

            robot.dashboard.displayPrintf(2, "Pose:%s", robot.robotDrive.driveBase.getFieldPosition());
        }
        //
        // Other subsystems.
        //
        if (robot.arm != null)
        {
            double armPower = operatorGamepad.getRightStickY(true)*armPowerScale;

            if (armManualOverride)
            {
                robot.arm.setPower(armPower);
            }
            else
            {
                robot.arm.setPidPower(armPower);
            }
            robot.dashboard.displayPrintf(
                3, "Arm: Pow=%.1f,Pos=%.1f,Lim=(%b,%b)",
                armPower, robot.arm.getPosition(), robot.arm.isLowerLimitSwitchActive(),
                robot.arm.isUpperLimitSwitchActive());
        }

        if (robot.intake != null)
        {
            robot.dashboard.displayPrintf(
                4, "Intake: Pow=%.1f,sensor=%.2f", robot.intake.getPower(), robot.intake.getSensorValue());
        }

        if (robot.spinner != null)
        {
            robot.dashboard.displayPrintf(5, "Spinner: Pow=%.1f", robot.spinner.getMotorPower());
        }

        if (robot.odwDeployer != null)
        {
            robot.dashboard.displayPrintf(
                6, "odwDep: deployed=%s", robot.odwDeployer.isDeployed());
        }
    }   //slowPeriodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void driverButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                break;

            case FtcGamepad.GAMEPAD_B:
                break;

            case FtcGamepad.GAMEPAD_X:
                break;

            case FtcGamepad.GAMEPAD_Y:
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                if (pressed)
                {
                    // Toggle inverted drive.
                    invertedDrive = !invertedDrive;
                }
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                // Press and hold for slow drive.
                drivePowerScale = pressed? RobotParams.SLOW_DRIVE_POWER_SCALE: 1.0;
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (robot.odwDeployer != null && pressed)
                {
                    robot.odwDeployer.retract();
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (robot.odwDeployer != null && pressed)
                {
                    robot.odwDeployer.deploy();
                }
                break;
        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void operatorButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(
            7, "%s: %04x->%s", gamepad, button, pressed? "Pressed": "Released");

        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                if (robot.intake != null)
                {
                    robot.intake.cancelAutoAssist();    //cancel auto-assist if it is active.
                    robot.intake.setPower(pressed? RobotParams.INTAKE_POWER_DUMP: 0.0);
                }
                break;

            case FtcGamepad.GAMEPAD_B:
                if (robot.spinner != null)
                {
                    robot.spinner.set(pressed? RobotParams.SPINNER_POWER_RED: 0.0);
                }
                break;

            case FtcGamepad.GAMEPAD_X:
                if (robot.spinner != null)
                {
                    robot.spinner.set(pressed? RobotParams.SPINNER_POWER_BLUE: 0.0);
                }
                break;

            case FtcGamepad.GAMEPAD_Y:
                if (robot.intake != null)
                {
                    robot.intake.cancelAutoAssist();    //cancel auto-assist if it is active.
                    robot.intake.setPower(pressed? RobotParams.INTAKE_POWER_PICKUP: 0.0);
                }
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                armManualOverride = pressed;
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                if (robot.arm != null)
                {
                    armPowerScale = pressed? RobotParams.ARM_SLOW_POWER_SCALE: 1.0;
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (robot.arm != null && pressed)
                {
                    robot.arm.presetPositionUp();
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (robot.arm != null && pressed)
                {
                    robot.arm.presetPositionDown();
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                break;

            case FtcGamepad.GAMEPAD_BACK:
                if (robot.arm != null && pressed)
                {
                    robot.arm.zeroCalibrate();
                }
                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
