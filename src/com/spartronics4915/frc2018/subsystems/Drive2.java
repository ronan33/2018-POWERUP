package com.spartronics4915.frc2018.subsystems;

import java.util.Optional;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.Kinematics;
import com.spartronics4915.frc2018.RobotState;
import com.spartronics4915.frc2018.ShooterAimingParameters;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import com.spartronics4915.lib.util.Util;
import com.spartronics4915.lib.util.control.Lookahead;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.control.PathFollower;
import com.spartronics4915.lib.util.drivers.CANTalon4915Drive;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Twist2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This subsystem consists of the robot's drivetrain: 4 CIM motors, 4 talons,
 * one solenoid and 2 pistons to shift gears, and a Pigeon IMU board.
 * The Drive subsystem has several control methods including open loop,
 * velocity control, and position control. The Drive subsystem also has
 * several methods that handle automatic aiming, autonomous path driving, and
 * manual control.
 * 
 * @see Subsystem.java
 */
public class Drive2 extends Subsystem
{

    private static Drive2 mInstance = null;

    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
    private static final double kOpenLoopRampRate = .5;
    private static final double kOpenLoopNominalOutput = 0.0; // fwd & rev
    private static final double kOpenLoopPeakOutput = .5; // fwd: .5, rev: -.5

    public static Drive2 getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new Drive2();
        }
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState
    {
        OPEN_LOOP, // open loop voltage control
        VELOCITY_SETPOINT, // velocity PID control
        PATH_FOLLOWING, // used for autonomous driving
        AIM_TO_GOAL, // turn to face the boiler
        TURN_TO_HEADING, // turn in place
        DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
        DRIVE_TOWARDS_GOAL_APPROACH // drive forwards until we are at optimal shooting distance
    }

    // Control states
    private DriveControlState mDriveControlState;

    // Hardware
    private CANTalon4915Drive mDrive = null;

    // Controllers
    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;

    // These gains get reset below!!
    private Rotation2d mTargetHeading = new Rotation2d();
    private Path mCurrentPath = null;

    // Hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private boolean mIsOnTarget = false;
    private boolean mIsApproaching = false;

    // Logging
    private final ReflectingCSVWriter<PathFollower.DebugOutput> mCSVWriter;

    // mLoop not registered when we're not initialized
    private final Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Drive2.this)
            {
                setOpenLoop(DriveSignal.NEUTRAL);
                setBrakeMode(false);
                setVelocitySetpoint(0, 0);
                if (!mDrive.hasIMU())
                {
                    logError("IMU in non-ready state. Is it plugged in?");
                    return;
                }
                mDrive.setGyroAngle(0);
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Drive2.this)
            {
                switch (mDriveControlState)
                {
                    case OPEN_LOOP:
                        return;
                    case VELOCITY_SETPOINT:
                        return;
                    case PATH_FOLLOWING:
                        if (mPathFollower != null)
                        {
                            updatePathFollower(timestamp);
                            mCSVWriter.add(mPathFollower.getDebug());
                        }
                        return;
                    case TURN_TO_HEADING:
                        updateTurnToHeading(timestamp);
                        return;
                    case DRIVE_TOWARDS_GOAL_COARSE_ALIGN:
                        updateDriveTowardsGoalCoarseAlign(timestamp);
                        return;
                    case DRIVE_TOWARDS_GOAL_APPROACH:
                        updateDriveTowardsGoalApproach(timestamp);
                        return;
                    default:
                        logError("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            stop();
            mCSVWriter.flush();
        }
    };

    private Drive2()
    {
        // encoder phase must match output sense or PID will spiral out of control
        mDrive = new CANTalon4915Drive(Constants.kDriveWheelDiameterInches,
                Constants.kLeftDriveMasterId,
                Constants.kLeftDriveSlaveId,
                Constants.kRightDriveMasterId,
                Constants.kRightDriveSlaveId,
                Constants.kDriveIMUTalonId,
                CANTalon4915Drive.Config.kLeftNormalRightInverted);

        if (mDrive.isInitialized())
        {
            reloadGains();
            mIsHighGear = false;
            setHighGear(true);
            mDrive.beginOpenLoop(kOpenLoopRampRate,
                    kOpenLoopNominalOutput, kOpenLoopPeakOutput);
            // Path Following stuff
            if (!mDrive.hasIMU())
                logError("Could not detect the IMU. Is it plugged in?");
            mIsBrakeMode = true;
            mDrive.enableBraking(mIsBrakeMode);
            logInitialized(true);
        }
        else
        {
            logInitialized(false);
        }

        mCSVWriter = new ReflectingCSVWriter<PathFollower.DebugOutput>(
                "/home/lvuser/PATH-FOLLOWER-LOGS.csv",
                PathFollower.DebugOutput.class);
    }

    @Override
    public void registerEnabledLoops(Looper in)
    {
        if (!this.isInitialized())
            return;

        in.register(mLoop);
    }

    /**
     * drives with open-loop control mode, called frequently!
     */
    public synchronized void setOpenLoop(DriveSignal signal)
    {
        if (!this.isInitialized())
            return;
        if (mDriveControlState != DriveControlState.OPEN_LOOP)
        {
            configureTalonsForOpenLoop();
        }
        mDrive.driveOpenLoop(signal.getLeft(), signal.getRight());
    }

    public boolean isHighGear()
    {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear)
    {
        // XXX: Dead code
        if (wantsHighGear != mIsHighGear)
        {
            mIsHighGear = wantsHighGear;
        }
    }

    public boolean isBrakeMode()
    {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean s)
    {
        if (!this.isInitialized())
            return;
        if (mIsBrakeMode != s)
        {
            mIsBrakeMode = s;
            mDrive.enableBraking(s);
        }
    }

    @Override
    public synchronized void stop()
    {
        this.setOpenLoop(new DriveSignal(0.0, 0.0));
    }

    @Override
    public void outputToSmartDashboard()
    {
        if (!this.isInitialized())
            return;
        mDrive.outputToSmartDashboard(usesTalonVelocityControl(mDriveControlState));
        synchronized (this)
        {
            if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null)
            {
                SmartDashboard.putNumber("drive CTE", mPathFollower.getCrossTrackError());
                SmartDashboard.putNumber("drive ATE", mPathFollower.getAlongTrackError());
            }
            else
            {
                SmartDashboard.putNumber("drive CTE", 0.0);
                SmartDashboard.putNumber("drive ATE", 0.0);
            }
        }
        SmartDashboard.putBoolean("drive on target", isOnTarget());
    }

    public synchronized void resetEncoders()
    {
        if (!this.isInitialized())
            return;
        mDrive.resetEncoders(false);
    }

    @Override
    public void zeroSensors()
    {
        if (!this.isInitialized())
            return;
        mDrive.resetEncoders(true);
    }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    public synchronized void setVelocitySetpoint(double left_inches_per_sec,
            double right_inches_per_sec)
    {
        if (!this.isInitialized())
            return;
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    private void configureTalonsForOpenLoop()
    {
        if (!this.isInitialized())
            return;
        logNotice("beginOpenLoop");
        mDrive.beginOpenLoop(kOpenLoopRampRate, kOpenLoopNominalOutput, kOpenLoopPeakOutput);
        setBrakeMode(false);
        mDriveControlState = DriveControlState.OPEN_LOOP;
    }

    /**
     * Configures talons for velocity control
     */
    private void configureTalonsForSpeedControl()
    {
        if (!this.isInitialized())
            return;
        if (!usesTalonVelocityControl(mDriveControlState))
        {
            // We entered a velocity control state.
            logNotice("beginSpeedControl");
            mDrive.beginClosedLoopVelocity(kHighGearVelocityControlSlot,
                    Constants.kDriveHighGearNominalOutput);
            setBrakeMode(true);
        }
    }

    /**
     * Configures talons for position control
     */
    private void configureTalonsForPositionControl()
    {
        if (!this.isInitialized())
            return;
        if (!usesTalonPositionControl(mDriveControlState))
        {
            // We entered a position control state.
            logNotice("beginPositionControl");
            mDrive.beginClosedLoopPosition(kLowGearPositionControlSlot,
                    Constants.kDriveLowGearNominalOutput,
                    Constants.kDriveLowGearMaxVelocity,
                    Constants.kDriveLowGearMaxAccel);

            // XXX: need to disable voltage compensation ramp rate
            // mLeftMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);
            // mRightMaster.setVoltageCompensationRampRate(Constants.kDriveVoltageCompensationRampRate);

            setBrakeMode(true);
        }
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec,
            double right_inches_per_sec)
    {
        if (!this.isInitialized())
            return;
        if (usesTalonVelocityControl(mDriveControlState))
        {
            final double max_desired =
                    Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
            mDrive.driveVelocityInchesPerSec(left_inches_per_sec * scale,
                    right_inches_per_sec * scale);
        }
        else
        {
            logError("Hit a bad velocity control state");
            mDrive.driveVelocityInchesPerSec(0, 0); // XXX: should we just mDrive.stop()?
        }
    }

    /**
     * Adjust position setpoint (if already in position mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     */
    private synchronized void updatePositionSetpoint(double left_position_inches,
            double right_position_inches)
    {
        if (!this.isInitialized())
            return;
        if (usesTalonPositionControl(mDriveControlState))
        {
            mDrive.drivePositionInches(left_position_inches, right_position_inches);
        }
        else
        {
            logError("Hit a bad position control state");
            mDrive.stop();
        }
    }

    /**
     * Update the heading at which the robot thinks the boiler is.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateGoalHeading(double timestamp)
    {
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
        if (aim.isPresent())
        {
            mTargetHeading = aim.get().getRobotToGoal();
        }
    }

    /**
     * Turn the robot to a target heading.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     * 
     * This actually doesn't use the IMU... Only the wheel encoders and
     * kinematics.
     */
    private void updateTurnToHeading(double timestamp)
    {
        if (!this.isInitialized())
            return;
        final Rotation2d field_to_robot =
                mRobotState.getLatestFieldToVehicle().getValue().getRotation(); // We need the field frame because this is specified in field coordinates, not robot ones
        // Figure out the rotation necessary to turn to face the goal.
        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

        // Check if we are on target
        final double kGoalPosTolerance = 0.75; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
                && Math.abs(mDrive.getLeftVelocityInchesPerSec()) < kGoalVelTolerance
                && Math.abs(mDrive.getRightVelocityInchesPerSec()) < kGoalVelTolerance)
        {
            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            updatePositionSetpoint(mDrive.getLeftDistanceInches(), mDrive.getRightDistanceInches());
            return;
        }

        Kinematics.DriveVelocity wheel_delta = Kinematics
                .inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
        updatePositionSetpoint(wheel_delta.left + mDrive.getLeftDistanceInches(),
                wheel_delta.right + mDrive.getRightDistanceInches());
    }

    /**
     * Essentially does the same thing as updateTurnToHeading but sends the
     * robot into the DRIVE_TOWARDS_GOAL_APPROACH
     * state if it detects we are not at an optimal shooting range
     */
    private void updateDriveTowardsGoalCoarseAlign(double timestamp)
    {
        if (!this.isInitialized())
            return;
        updateGoalHeading(timestamp);
        updateTurnToHeading(timestamp);
        mIsApproaching = true;
        if (mIsOnTarget)
        {
            // Done coarse alignment.

            Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
            if (aim.isPresent())
            {
                final double distance = aim.get().getRange();

                if (distance < Constants.kShooterOptimalRangeCeiling &&
                        distance > Constants.kShooterOptimalRangeFloor)
                {
                    // Don't drive, just shoot.
                    mDriveControlState = DriveControlState.AIM_TO_GOAL;
                    mIsApproaching = false;
                    mIsOnTarget = false;
                    updatePositionSetpoint(mDrive.getLeftDistanceInches(),
                            mDrive.getRightDistanceInches());
                    return;
                }
            }

            mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH;
            mIsOnTarget = false;
        }
    }

    /**
     * Drives the robot straight forwards until it is at an optimal shooting
     * distance. Then sends the robot into the
     * AIM_TO_GOAL state for one final alignment
     */
    private void updateDriveTowardsGoalApproach(double timestamp)
    {
        if (!this.isInitialized())
            return;
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
        mIsApproaching = true;
        if (aim.isPresent())
        {
            final double distance = aim.get().getRange();
            double error = 0.0;
            if (distance < Constants.kShooterOptimalRangeFloor)
            {
                error = distance - Constants.kShooterOptimalRangeFloor;
            }
            else if (distance > Constants.kShooterOptimalRangeCeiling)
            {
                error = distance - Constants.kShooterOptimalRangeCeiling;
            }
            final double kGoalPosTolerance = 1.0; // inches
            if (Util.epsilonEquals(error, 0.0, kGoalPosTolerance))
            {
                // We are on target. Switch back to auto-aim.
                mDriveControlState = DriveControlState.AIM_TO_GOAL;
                RobotState.getInstance().resetVision();
                mIsApproaching = false;
                updatePositionSetpoint(mDrive.getLeftDistanceInches(),
                        mDrive.getRightDistanceInches());
                return;
            }
            updatePositionSetpoint(mDrive.getLeftDistanceInches() + error,
                    mDrive.getRightDistanceInches() + error);
        }
        else
        {
            updatePositionSetpoint(mDrive.getLeftDistanceInches(),
                    mDrive.getRightDistanceInches());
        }
    }

    /**
     * Called periodically when the robot is in path following mode. Updates the
     * path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity
     * setpoints.
     */
    private void updatePathFollower(double timestamp)
    {
        if (!this.isInitialized())
            return;
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(),
                RobotState.getInstance().getPredictedVelocity().dx);

        if (!mPathFollower.isFinished())
        {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        }
        else
        {
            updateVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isOnTarget()
    {
        // return true;
        return mIsOnTarget;
    }

    public synchronized boolean isAutoAiming()
    {
        return mDriveControlState == DriveControlState.AIM_TO_GOAL;
    }

    /**
     * Configures the drivebase for auto aiming
     */
    public synchronized void setWantAimToGoal()
    {
        if (mDriveControlState != DriveControlState.AIM_TO_GOAL)
        {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.AIM_TO_GOAL;
            updatePositionSetpoint(mDrive.getLeftDistanceInches(),
                    mDrive.getRightDistanceInches());
            mTargetHeading = Rotation2d.fromDegrees(mDrive.getGyroAngle());
        }
        setHighGear(false);
    }

    /**
     * Configures the drivebase for auto driving
     */
    public synchronized void setWantDriveTowardsGoal()
    {
        if (mDriveControlState != DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN &&
                mDriveControlState != DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH &&
                mDriveControlState != DriveControlState.AIM_TO_GOAL)
        {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN;
            updatePositionSetpoint(mDrive.getLeftDistanceInches(),
                    mDrive.getRightDistanceInches());
            mTargetHeading = Rotation2d.fromDegrees(mDrive.getGyroAngle());
        }
        setHighGear(false);
    }

    /**
     * Configures the drivebase to turn to a desired heading
     */
    public synchronized void setWantTurnToHeading(Rotation2d heading)
    {
        if (mDriveControlState != DriveControlState.TURN_TO_HEADING)
        {
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.TURN_TO_HEADING;
            updatePositionSetpoint(mDrive.getLeftDistanceInches(),
                    mDrive.getRightDistanceInches());
        }
        if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3)
        {
            mTargetHeading = heading;
            mIsOnTarget = false;
        }
        setHighGear(false);
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed)
    {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING)
        {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv,
                            Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance,
                            Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        }
        else
        {
            setVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isDoneWithPath()
    {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null)
        {
            return mPathFollower.isFinished();
        }
        else
        {
            logError("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath()
    {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null)
        {
            mPathFollower.forceFinish();
        }
        else
        {
            logError("Robot is not in path following mode");
        }
    }

    public boolean isApproaching()
    {
        return mIsApproaching;
    }

    public synchronized boolean isDoneWithTurn()
    {
        if (mDriveControlState == DriveControlState.TURN_TO_HEADING)
        {
            return mIsOnTarget;
        }
        else
        {
            logError("Robot is not in turn to heading mode");
            return false;
        }
    }

    public synchronized boolean hasPassedMarker(String marker)
    {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null)
        {
            return mPathFollower.hasPassedMarker(marker);
        }
        else
        {
            logError("Robot is not in path following mode");
            return false;
        }
    }

    // fill our two slots with gains...
    public synchronized void reloadGains()
    {
        if (!isInitialized())
            return;

        mDrive.reloadGains(kLowGearPositionControlSlot,
                Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate);

        mDrive.reloadGains(kHighGearVelocityControlSlot,
                Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate);

        // nb: motionMagic velocity and accel aren't slot-based, so should be established
        //   when we enter the control mode.

    }

    @Override
    public void writeToLog()
    {
        mCSVWriter.write();
    }

    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state)
    {
        if (state == DriveControlState.VELOCITY_SETPOINT
                || state == DriveControlState.PATH_FOLLOWING)
        {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlState state)
    {
        if (state == DriveControlState.AIM_TO_GOAL ||
                state == DriveControlState.TURN_TO_HEADING ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH)
        {
            return true;
        }
        return false;
    }

    public boolean checkSystem()
    {
        if (!isInitialized())
        {
            logWarning("can't check un-initialized system");
            return false;
        }
        return mDrive.checkSystem();
    }
}
