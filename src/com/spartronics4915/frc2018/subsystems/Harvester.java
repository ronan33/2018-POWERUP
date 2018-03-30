package com.spartronics4915.frc2018.subsystems;

import com.spartronics4915.frc2018.Constants;
import com.spartronics4915.frc2018.loops.Loop;
import com.spartronics4915.frc2018.loops.Looper;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.drivers.LazySolenoid;
import com.spartronics4915.lib.util.drivers.SpartIRSensor;
import com.spartronics4915.lib.util.drivers.TalonSRX4915;
import com.spartronics4915.lib.util.drivers.TalonSRX4915Factory;
import edu.wpi.first.wpilibj.Timer;

/**
 * The harvester is a set of two collapsible rollers that pull in and hold
 * a cube. This cube is then in a position where it can be picked up by the
 * articulated grabber.
 */
public class Harvester extends Subsystem
{

    private static Harvester sInstance = null;
    private static final boolean kSolenoidOpen = true;
    private static final boolean kSolenoidClose = false;
    private static final double kCloseTimePeriod = 1.0;
    private static final double kEjectTimePeriod = 2.3;
    private static final double kHarvestPercentOutput = 1.0;
    private static final double kEjectPercentOutput = -1.0;
    
    public static Harvester getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new Harvester();
        }
        return sInstance;
    }

    public enum SystemState
    {
        IDLING,
        HARVESTING,
        EJECTING,
        DISABLING,
    }

    public enum WantedState
    {
        IDLE,
        HARVEST,
        EJECT,
        DISABLE,
    }

    private SystemState mSystemState = SystemState.DISABLING;
    private WantedState mWantedState = WantedState.DISABLE;
    private SpartIRSensor mCubeHeldSensor = null;
    private TalonSRX4915 mMotorRight = null;
    private TalonSRX4915 mMotorLeft = null;
    private Timer mTimer;

    // Actuators and sensors should be initialized as private members with a value of null here

    private Harvester()
    {
        boolean success = true;

        // Instantiate your actuator and sensor objects here
        // If !mMyMotor.isValid() then success should be set to false

        try
        {
            mCubeHeldSensor = new SpartIRSensor(Constants.kGrabberCubeDistanceRangeFinderId);
            mMotorRight = TalonSRX4915Factory.createDefaultMotor(Constants.kHarvesterRightMotorId); // change value of motor
            mMotorLeft = TalonSRX4915Factory.createDefaultSlave(Constants.kHarvesterLeftMotorId, 
                    Constants.kHarvesterRightMotorId, true); // true means motor inverted
            mMotorRight.configOutputPower(true, 0.5, 0, 1.0, 0, -1.0);
            mMotorLeft.configOutputPower(true, 0.5, 0, 1.0, 0, -1.0);
            mMotorRight.setInverted(false);
            mTimer = new Timer();

            if (!mMotorRight.isValid())
            {
                logError("Right Motor is invalid");
                success = false;
            }
        }
        catch (Exception e)
        {
            logError("Couldn't instantiate hardware objects");
            Logger.logThrowableCrash(e);
            success = false;
        }

        logInitialized(success);
    }

    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Harvester.this)
            {
                if (mSystemState == SystemState.DISABLING)
                    mSystemState = SystemState.IDLING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Harvester.this)
            {
                SystemState newState; // calls the wanted handle case for the given systemState
                switch (mSystemState)
                {
                    case IDLING:
                        mMotorRight.set(0.0);
                        newState = defaultStateTransfer();
                        break;
                    case HARVESTING:
                        mMotorRight.set(kHarvestPercentOutput);
                        if (mTimer.hasPeriodPassed(kCloseTimePeriod))
                        {
                            setWantedState(WantedState.IDLE);
                        }
                        newState = defaultStateTransfer();
                        break;
                    case EJECTING:
                        mMotorRight.set(kEjectPercentOutput);
                        if (mTimer.hasPeriodPassed(kEjectTimePeriod))
                        { //Cube is gone!  Transition to Open (turn off motor) to prevent damage
                            setWantedState(WantedState.IDLE);
                        }
                        newState = defaultStateTransfer();
                        break;
                    case DISABLING:
                        mMotorRight.set(0.0);
                        newState = SystemState.IDLING;
                        break;
                    default:
                        mMotorRight.set(0.0);
                        newState = SystemState.IDLING;
                        break;
                }
                if (newState != mSystemState)
                {
                    logInfo("Harvester state from " + mSystemState + "to" + newState);
                    dashboardPutState(mSystemState.toString());
                    mSystemState = newState;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (Harvester.this)
            {
                stop();
            }
        }

    };

    private SystemState defaultStateTransfer() // transitions the systemState given what the wantedState is
    {

        switch (mWantedState)
        {
            case IDLE:
                return SystemState.IDLING;
            case HARVEST:
                return SystemState.HARVESTING;
            case EJECT:
                return SystemState.EJECTING;
            default:
                return mSystemState;
        }
    }
    
    public WantedState getWantedState()
    {
        return mWantedState;
    }

    public void setWantedState(WantedState wantedState)
    {
        if (wantedState == WantedState.HARVEST || wantedState == WantedState.IDLE)
        {
            logNotice("TIMER SET---------------------------------------------");
            mTimer.reset();
            mTimer.start();
        }
        mWantedState = wantedState;
        dashboardPutWantedState(mWantedState.toString());
    }

    public boolean atTarget()
    {
        boolean t = false;
        switch (mSystemState)
        {
            case IDLING:
                if (mWantedState == WantedState.IDLE)
                    t = true;
                break;
            case HARVESTING:
                if (mWantedState == WantedState.HARVEST)
                    t = true;
                break;
            case EJECTING:
                if (mWantedState == WantedState.EJECT)
                    t = true;
                break;
            case DISABLING:
                if (mWantedState == WantedState.DISABLE)
                    t = true;
                break;
            default:
                t = false;
                break;
        }
        return t;
    }

    private boolean isCubeHeld()
    {
        return mCubeHeldSensor.isTargetAcquired();
    }

    @Override
    public void outputToSmartDashboard()
    {
        if(!isInitialized()) return;
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
        dashboardPutBoolean("IRSensor CubeHeld", isCubeHeld());
        dashboardPutNumber("MotorRight", mMotorRight.get());
        dashboardPutNumber("Cube Distance: ", mCubeHeldSensor.getDistance());
    }

    @Override
    public synchronized void stop()
    {
        mSystemState = SystemState.DISABLING;
        mWantedState = WantedState.DISABLE;
        mMotorRight.set(0.0);
    }

    @Override
    public void zeroSensors()
    {
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    @Override
    public boolean checkSystem(String variant)
    {
        boolean success = true;
        if (!isInitialized())
        {
            logWarning("can't check un-initialized system");
            return false;
        }
        logNotice("checkSystem (" + variant + ") ------------------");

        try
        {
            boolean allTests = variant.equalsIgnoreCase("all") || variant.equals("");
            if (variant.equals("basic") || allTests)
            {
                logNotice("basic check ------");
                logNotice("  mMotorRight:\n" + mMotorRight.dumpState());
                logNotice("  isCubeHeld: " + isCubeHeld());
            }
            if (variant.equals("solenoid") || allTests)
            {
                logNotice("solenoid check ------");
                logNotice("on 4s");
                Timer.delay(4.0);
                logNotice("  isCubeHeld: " + isCubeHeld());
                logNotice("off");
            }
            if (variant.equals("motors") || allTests)
            {
                logNotice("motors check ------");
                logNotice("open arms (2s)");
                Timer.delay(2.0);

                logNotice("left motor fwd .5 (4s)"); // in
                Timer.delay(4.0);

                logNotice("right motor fwd .5 (4s)");
                mMotorRight.set(.5);
                Timer.delay(4.0);
                logNotice("  current: " + mMotorRight.getOutputCurrent());
                mMotorRight.set(0);

                logNotice("both motors rev .5 (4s)"); // out
                mMotorRight.set(-.5);
                Timer.delay(4.0);
                logNotice("  right current: " + mMotorRight.getOutputCurrent());
                mMotorRight.set(0);

                Timer.delay(.5); // let motors spin down
            }
            if (variant.equals("IRSensor") || allTests)
            {
                logNotice("SensorCheck");
                logNotice("Is Cube Held?");
                logNotice("Cube Distance: " + mCubeHeldSensor.getDistance());
            }
        }
        catch (Throwable e)
        {
            success = false;
            logException("checkSystem", e);
        }

        logNotice("--- finished ---------------------------");
        return success;
    }
}
