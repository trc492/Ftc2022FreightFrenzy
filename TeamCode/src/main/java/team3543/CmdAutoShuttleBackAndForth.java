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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

class CmdAutoShuttleBackAndForth implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoShuttleBackAndForth";
    //should be 8 but last round we should only need 5-6 sec, full power holonomic drive at the end
    private static final double CYCLE_TRIP_TIME = 5.0;

    private enum State
    {
        START_DELAY,
        DRIVE_TO_ALLIANCE_SHIPPING_HUB,
        DUMP_FREIGHT,

        PREP_FOR_DRIVE_INTO_WAREHOUSE,
        ALIGN_TO_WALL,

        DRIVE_INTO_WAREHOUSE,
        LOOK_FOR_FREIGHT,
        PICK_UP_FREIGHT_FROM_WAREHOUSE,
        DETERMINE_ROUND_TRIP_OR_DONE,
        RETRY_PICKUP,

//        BACKUP,
//        REALIGN_TO_WALL,
//        BACK_OUT_OF_WAREHOUSE,

        DRIVE_OUT_OF_WAREHOUSE_TO_SHIPPING_HUB,

        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent pickupEvent;
    private final TrcStateMachine<State> sm;
    private final TrcPose2D redLookingPos, blueLookingPos;
    private int duckPosition = 0;
    private final boolean useVisionForPickup = false;
    private TrcVisionTargetInfo<?> freightInfo;
    private double pickupHeadingInc = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAutoShuttleBackAndForth(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        pickupEvent = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);
        redLookingPos = robot.robotDrive.pathPoint(1.65, -2.7, 90.0);
        blueLookingPos = robot.robotDrive.pathPoint(1.65, 2.7, 90.0);
        sm.start(State.START_DELAY);
    }   //CmdAutoShuttleBackAndForth

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        robot.robotDrive.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        //
        // Assume narrow robot with width less than 14 inches.
        //
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            boolean traceState = true;
            String msg;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    robot.robotDrive.driveBase.setFieldPosition(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ?
                            RobotParams.STARTPOS_RED_FAR : RobotParams.STARTPOS_BLUE_FAR);
                    // Call vision at the beginning to figure out the position of the duck.
                    if (robot.vision != null)
                    {
                        duckPosition = robot.vision.getLastDuckPosition();
                        msg = "Duck found at position " + duckPosition;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }

                    if (duckPosition == 0)
                    {
                        //
                        // We still can't see the duck, default to level 3.
                        //
                        duckPosition = 3;
                        msg = "No duck found, default to position " + duckPosition;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }
                    robot.arm.setPresetPosition(duckPosition);
                    //
                    // Do start delay if any.
                    //
                    if (autoChoices.startDelay < 1.0)
                    {
                        autoChoices.startDelay = 1.0;
                    }

                    if (autoChoices.startDelay == 0.0)
                    {
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
                        break;
                    }

                case DRIVE_TO_ALLIANCE_SHIPPING_HUB:
                    // Drive to alliance shipping hub. We are coming from starting position or somewhere near carousel.
                    // Note: the smaller the number the closer to the hub.
                    double hubHeading, distanceToHub;
                    double hubX, hubY;
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        hubHeading = -30.0;
                        distanceToHub = duckPosition == 3? 0.7: duckPosition == 2? 0.9: 0.85;
                        // alliance hub center location in tile unit.
                        hubX = -0.5;
                        hubY = -1.0;
                    }
                    else
                    {
                        hubHeading = 180.0 + 30.0;
                        distanceToHub = duckPosition == 3? 0.7: duckPosition == 2? 0.9: 0.85;
                        // alliance hub center location in tile unit.
                        hubX = -0.5;
                        hubY = 1.0;
                    }
                    // calculate the hub perimeter point the robot needs to be at.
                    hubX -= distanceToHub*Math.sin(Math.toRadians(hubHeading));
                    hubY -= distanceToHub*Math.cos(Math.toRadians(hubHeading));

                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        // Add one extra point to make sure it doesn't overshoot the turn.
                        robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(hubX, hubY, hubHeading));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(hubX, hubY, hubHeading));
                    }
                    // Raise arm to the detected duck level at the same time.
                    robot.arm.setPresetPosition(duckPosition);
                    // After we dump the freight to the right level for the bonus, any subsequent dumps will be to
                    // the top.
                    duckPosition = 3;
                    sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    break;

                case DUMP_FREIGHT:
                    // Dumps the freight, when done signals event and goes to next state
                    robot.intake.setPower(RobotParams.INTAKE_POWER_DUMP, RobotParams.INTAKE_DUMP_TIME, event);
                    sm.waitForSingleEvent(event, State.PREP_FOR_DRIVE_INTO_WAREHOUSE);
                    break;

                case PREP_FOR_DRIVE_INTO_WAREHOUSE:
                    robot.arm.setPresetPosition(0.5, 0);
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(0.5, -2.5, 90.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(0.5, 2.5, 90.0));
                    }
                    sm.waitForSingleEvent(event, State.ALIGN_TO_WALL);
                    break;

                case ALIGN_TO_WALL:

                    robot.robotDrive.driveBase.holonomicDrive(
                         autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.3: -0.3,
                         0.0, 0.0);
                    timer.set(0.8, event);
                    sm.waitForSingleEvent(event, State.DRIVE_INTO_WAREHOUSE);
                    break;

                case DRIVE_INTO_WAREHOUSE:
                    // Fire and forget with lowering the arm.
                    robot.robotDrive.driveBase.stop();
                    double relocalizedY =
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                            -(RobotParams.HALF_FIELD_INCHES - RobotParams.ROBOT_WIDTH/2.0):
                            (RobotParams.HALF_FIELD_INCHES - RobotParams.ROBOT_WIDTH/2.0);
                    robot.robotDrive.driveBase.setFieldPosition(
                        new TrcPose2D(robot.robotDrive.driveBase.getXPosition(),
                        relocalizedY, 90.0));
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotDrive.driveBase.holonomicDrive(0.0, 0.5, 0.0);
                    timer.set(1, event);
//                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
//                    {
//                        robot.robotDrive.purePursuitDrive.start(
//                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                            robot.robotDrive.pathPoint(1.7, robot.robotDrive.driveBase.getYPosition(), 90.0),
//                            redLookingPos);
//                    }
//                    else
//                    {
//                        robot.robotDrive.purePursuitDrive.start(
//                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                            robot.robotDrive.pathPoint(1.7, 2.65, 90.0),
//                            blueLookingPos);
//                    }
                    sm.waitForSingleEvent(event, State.PICK_UP_FREIGHT_FROM_WAREHOUSE);
                    break;

//                case LOOK_FOR_FREIGHT:
//                    if (useVisionForPickup)
//                    {
//                        freightInfo = robot.vision.getClosestFreightInfo();
//                        if (freightInfo != null)
//                        {
//                            if (robot.blinkin != null)
//                            {
//                                robot.blinkin.setPatternState(Vision.sawTarget, true);
//                            }
//                            sm.setState(State.PICK_UP_FREIGHT_FROM_WAREHOUSE);
//                        }
//                        else if (expireTime == null)
//                        {
//                            expireTime = TrcUtil.getCurrentTime() + 2.0;
//                        }
//                        else if (TrcUtil.getCurrentTime() > expireTime)
//                        {
//                            // if we cant see the freight, disable visionForPickup because we dont want to waste
//                            // any more time looking for freight if vision is not working well.
//                            useVisionForPickup = false;
//                            sm.setState(State.PICK_UP_FREIGHT_FROM_WAREHOUSE);
//                        }
//                    }
//                    else
//                    {
//                        sm.setState(State.PICK_UP_FREIGHT_FROM_WAREHOUSE);
//                    }
//                    break;

                case PICK_UP_FREIGHT_FROM_WAREHOUSE:
                    // If there is not enough time left in autonomous, we go to done because we are already in the
                    // warehouse timeout is timeleft - cycletriptime, 3sec(roughly time it takes to jam)
                    robot.intake.autoAssist(RobotParams.INTAKE_POWER_PICKUP, pickupEvent, null, 0.0);
                    // Keep running drive base until next event is signaled
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                    // If we are using vision, drive to the target
                    if (useVisionForPickup)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), true,
                            new TrcPose2D(
                                freightInfo.distanceFromCamera.x, freightInfo.distanceFromCamera.y,
                                freightInfo.horizontalAngle));
                    }
                    else if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            //we could try 2.7 and see what happens, makes backing out easier
                            robot.robotDrive.pathPoint(2.6, -2.6, 90.0 - pickupHeadingInc));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(2.6, 2.6, 90.0 + pickupHeadingInc));
                    }
                    // Event is signaled by intake when robot picked up a block  or pure pursuit drive is done.
                    sm.addEvent(event);
                    sm.addEvent(pickupEvent);
                    sm.waitForEvents(State.DETERMINE_ROUND_TRIP_OR_DONE);
                    break;

                case DETERMINE_ROUND_TRIP_OR_DONE:
                    // If intake has freight and there are more than round trip time left.
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.intake.setPower(0.0);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    //maybe take this out? i think it screws up backing up and trying again
                    //robot.robotDrive.purePursuitDrive.setStallDetectionEnabled(false);
                    if (30.0 - elapsedTime > CYCLE_TRIP_TIME)
                    {
                        if (robot.intake.hasObject())
                        {
                            if (robot.blinkin != null)
                            {
                                robot.blinkin.setPatternState(Vision.GOT_TARGET, true);
                            }
                            //if it has freight, keep running intake so block doesnt fall out
                            robot.intake.setPower(RobotParams.INTAKE_POWER_PICKUP);
                            //next state is driving out of warehouse
                            sm.setState(State.DRIVE_OUT_OF_WAREHOUSE_TO_SHIPPING_HUB);
                            //sm.setState(State.BACKUP); - new untested algo
                        }
                        else
                        {
                            //case where we jam blocks and don't pick anything up
                            sm.setState(State.RETRY_PICKUP);
                        }
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case RETRY_PICKUP:
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false, redLookingPos);
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false, blueLookingPos);
                    }
                    pickupHeadingInc +=5.0;
                    sm.waitForSingleEvent(event, State.PICK_UP_FREIGHT_FROM_WAREHOUSE);
                    break;

                //new untested backing out
//                case BACKUP:
//                    if (autoChoices.alliance==FtcAuto.Alliance.RED_ALLIANCE)
//                    {
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(1.3, -2.6,  90.0));
//                    }
//                    else{
//                        robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.pathPoint(1.3, 2.6,  90.0));
//                    }
//                    sm.waitForSingleEvent(event, State.REALIGN_TO_WALL);
//                    break;
//
//                case REALIGN_TO_WALL:
//                    // CodeReview: half a second is way too long especially your pure pursuit point was 2.6 in Y.
//                    timer.set(0.5, event);
//                    if(autoChoices.alliance ==FtcAuto.Alliance.RED_ALLIANCE)
//                    {
//                        robot.robotDrive.driveBase.holonomicDrive(0.3, 0.0, 0.0);
//                    }
//                    else
//                    {
//                        robot.robotDrive.driveBase.holonomicDrive(-0.3, 0.0, 0.0);
//                    }
//                    sm.waitForSingleEvent(event, State.BACK_OUT_OF_WAREHOUSE);
//                    break;
//
//                case BACK_OUT_OF_WAREHOUSE:
//                    timer.set(1.0, event);
//                    robot.robotDrive.driveBase.holonomicDrive(0.0, -0.3, 0.0);
//                    sm.waitForSingleEvent(event, State.DRIVE_TO_ALLIANCE_SHIPPING_HUB);
//                    break;

                case DRIVE_OUT_OF_WAREHOUSE_TO_SHIPPING_HUB:
                    distanceToHub = 1.8;
                    robot.arm.setPresetPosition(3);
                    if (autoChoices.alliance==FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(1.5, -2.65,  90.0),
                            robot.robotDrive.pathPoint(-0.4, -2.65,  90.0),
                            robot.robotDrive.pathPoint(-0.4, -2.5, 0.0),
                            robot.robotDrive.pathPoint(-0.3, -distanceToHub, 0.0));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.pathPoint(1.5, 2.7,  90.0),
                            robot.robotDrive.pathPoint(-0.4, 2.7,  90.0),
                            robot.robotDrive.pathPoint(-0.4, 2.5, 180.0),
                            robot.robotDrive.pathPoint(-0.3, distanceToHub, 180.0));
                    }
                    //next state is dumping
                    sm.waitForSingleEvent(event, State.DUMP_FREIGHT);
                    break;

                case DONE:
                    default:
                        //
                        // We are done, zero calibrate the arm will lower it.
                        //
                        robot.arm.zeroCalibrate();
                        cancel();
                        break;
            }

            if (traceState)
            {
                robot.globalTracer.traceStateInfo(
                    sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoShuttleBackAndForth
