package com.team254.frc2017;

import java.util.Arrays;
import java.util.Map;

import com.team254.frc2017.auto.AutoModeExecuter;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.loops.RobotStateEstimator;
import com.team254.frc2017.loops.VisionProcessor;
import com.team254.frc2017.paths.profiles.PathAdapter;
import com.team254.frc2017.subsystems.ConnectionMonitor;
import com.team254.frc2017.subsystems.Drive;
import com.team254.frc2017.subsystems.LED;
import com.team254.frc2017.subsystems.Superstructure;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.DelayedBoolean;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.math.RigidTransform2d;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The main robot class, which instantiates all robot parts and helper classes and initializes all loops. Some classes
 * are already instantiated upon robot startup; for those classes, the robot gets the instance as opposed to creating a
 * new object
 * 
 * After initializing all robot parts, the code sets up the autonomous and teleoperated cycles and also code that runs
 * periodically inside both routines.
 * 
 * This is the nexus/converging point of the robot code and the best place to start exploring.
 * 
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {
    // Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private LED mLED = LED.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;

    // Create subsystem manager
    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(Drive.getInstance(), Superstructure.getInstance(),
                    ConnectionMonitor.getInstance(), LED.getInstance()));

    // Initialize other helper objects
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private ControlBoardInterface mControlBoard = new XboxControlBoard();

    private Looper mEnabledLooper = new Looper();

//    private VisionServer mVisionServer = VisionServer.getInstance();

    private AnalogInput mCheckLightButton = new AnalogInput(Constants.kLEDOnId);

    private DelayedBoolean mDelayedAimButton;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mEnabledLooper.register(VisionProcessor.getInstance());
            mEnabledLooper.register(RobotStateEstimator.getInstance());

//            mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());

            AutoModeSelector.initAutoModeSelector();

            mDelayedAimButton = new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);
            // Force an true update now to prevent robot from running at start.
            mDelayedAimButton.update(Timer.getFPGATimestamp(), true);

            // Pre calculate the paths we use for auto.
            PathAdapter.calculatePaths();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        zeroAllSensors();
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set drivebase, intake and superstructure to correct
     * states). Then gets the correct auto mode from the AutoModeSelector
     * 
     * @see AutoModeSelector.java
     */
    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }

            zeroAllSensors();
            mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);

            mAutoModeExecuter = null;

            // Shift to high
            mDrive.setHighGear(true);
            mDrive.setBrakeMode(true);

            mEnabledLooper.start();
            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(AutoModeSelector.getSelectedAutoMode());
            mAutoModeExecuter.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        allPeriodic();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);
            // Shift to high
            mDrive.setHighGear(true);
            zeroAllSensors();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     * 
     * The code uses state machines to ensure that no matter what buttons the driver presses, the robot behaves in a
     * safe and consistent manner.
     * 
     * Based on driver input, the code sets a desired state for each subsystem. Each subsystem will constantly compare
     * its desired and actual states and act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
        try {
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                    !mControlBoard.getLowGear()));
            boolean wantLowGear = mControlBoard.getLowGear();
            mDrive.setHighGear(!wantLowGear);

            if (mControlBoard.getBlinkLEDButton()) {
                mLED.setWantedState(LED.WantedState.BLINK);
            }

            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            PathAdapter.calculatePaths();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        final double kVoltageThreshold = 0.15;
        if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
            mLED.setLEDOn();
        } else {
            mLED.setLEDOff();
        }

        zeroAllSensors();
        allPeriodic();
    }

    @Override
    public void testInit() {
        Timer.delay(0.5);

        boolean results = Drive.getInstance().checkSystem();
        // e.g. results &= Intake.getInstance().checkSystem();

        if (!results) {
            System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        } else {
            System.out.println("ALL SYSTEMS PASSED");
        }
    }

    @Override
    public void testPeriodic() {
    }

    /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        mRobotState.outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();

        ConnectionMonitor.getInstance().setLastPacketTime(Timer.getFPGATimestamp());
    }
}
