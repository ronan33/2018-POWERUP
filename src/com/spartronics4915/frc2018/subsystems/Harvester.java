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
    private static final double speed = 1.0;
    
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
        CLOSING,
        OPENING,
        HARVESTING,
        EJECTING,
        HUGGING,
        AUTOHARVESTING,
        DISABLING,
    }

    public enum WantedState
    {
        OPEN,
        HARVEST,
        EJECT,
        HUG,
        AUTOHARVEST,
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
                    Constants.kHarvesterRightMotorId, false); // change value of motor
            mMotorRight.configOutputPower(true, 0.5, 0, 0.75, 0, -0.75);
            mMotorLeft.configOutputPower(true, 0.5, 0, 0.75, 0, -0.75);
            mMotorRight.setInverted(true);
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
                    mSystemState = SystemState.CLOSING;
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
                    case CLOSING:
                        newState = handleClosing();
                        break;
                    case OPENING:
                        newState = handleOpening();
                        break;
                    case HARVESTING:
                        newState = handleHarvesting();
                        break;
                    case EJECTING:
                        newState = handleEjecting();
                        break;
                    case HUGGING:
                        newState = handleHugging();
                        break;
                    case AUTOHARVESTING:
                        newState = handleAutoHarvesting();
                        break;
                    case DISABLING:
                        newState = handleClosing();
                        break;
                    default:
                        newState = handleClosing();
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
            case OPEN:
                return SystemState.OPENING;
            case HARVEST:
                return SystemState.HARVESTING;
            case EJECT:
                return SystemState.EJECTING;
            case HUG:
                return SystemState.HUGGING;
            case AUTOHARVEST:
                return SystemState.AUTOHARVESTING;
            default:
                return mSystemState;
        }
    }

    private SystemState handleClosing()
    {
        //motors off and bars in
        mMotorRight.set(0.0);
        if (mWantedState == WantedState.OPEN || mWantedState ==  WantedState.AUTOHARVEST || mWantedState == WantedState.HARVEST || mWantedState == WantedState.EJECT)
        {
            return defaultStateTransfer(); //all defaultStateTransfers return the wanted state
        }
        return SystemState.CLOSING;
    }

    private SystemState handleOpening()
    {
        // due to mechanical stuck issue, run motors reverse, open bars and turn off motors after timeout
        mMotorRight.set(0.0);
        if (mWantedState == WantedState.HARVEST || mWantedState == WantedState.EJECT || mWantedState == WantedState.AUTOHARVEST)
        {
            return defaultStateTransfer();
        }
        // if timeout reached, turn off motors
        if (!mTimer.hasPeriodPassed(kCloseTimePeriod))
        {
            //mMotorLeft.set(speed);
            //mMotorRight.set(speed);
        }
        return SystemState.OPENING;
    }
    
    private SystemState handleAutoHarvesting()
    {
        mMotorRight.set(0.0);
        //Checks if cube is held and transitions to Harvest when there is
        if (isCubeHeld())
        {
            setWantedState(WantedState.HARVEST);
            return SystemState.HARVESTING;
        }
        else
        {
            return defaultStateTransfer();
        }
    }

    private SystemState handleHarvesting()
    {
        //motors on forward and bars closing, hug when cube is gone
        mMotorRight.set(-speed);
        if (mTimer.hasPeriodPassed(kCloseTimePeriod))
        {
            setWantedState(WantedState.HUG);
            return defaultStateTransfer(); // checks if cube is in the robot and will transitions to hugging when the cube is fully in
        }
        else
        {
            return defaultStateTransfer();
        }
    }

    private SystemState handleEjecting()
    {
        //motors in reverse and bars closing, close when cube is gone
        mMotorRight.set(speed);
        if (mTimer.hasPeriodPassed(kEjectTimePeriod))
        { //Cube is gone!  Transition to Open (turn off motor) to prevent damage
            setWantedState(WantedState.OPEN);
            return defaultStateTransfer();
        }
        else
        {
            return defaultStateTransfer();
        }
    }

    private SystemState handleHugging()
    {
        //motors off and bars closing go to closed when cube is gone
        mMotorRight.set(0.0);
        if (mWantedState == WantedState.OPEN || mWantedState == WantedState.EJECT || mWantedState == WantedState.HARVEST)
        {
            return defaultStateTransfer();
        }
        return SystemState.HUGGING;
    }
    
    public WantedState getWantedState()
    {
        return mWantedState;
    }

    public void setWantedState(WantedState wantedState)
    {
        if (wantedState == WantedState.HARVEST || wantedState == WantedState.OPEN)
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
            case OPENING:
                if (mWantedState == WantedState.OPEN)
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
            case HUGGING:
                if (mWantedState == WantedState.HUG)
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
