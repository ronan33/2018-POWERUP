package com.spartronics4915.frc2018.paths;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2018.paths.PathBuilder.Waypoint;
import com.spartronics4915.lib.util.control.Path;
import com.spartronics4915.lib.util.math.RigidTransform2d;
import com.spartronics4915.lib.util.math.Rotation2d;
import com.spartronics4915.lib.util.math.Translation2d;

public class DriveToFarSwitchFromBPath implements PathContainer {

    ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

    public DriveToFarSwitchFromBPath()
    {
        sWaypoints.add(new Waypoint(18,166,0,0));
        sWaypoints.add(new Waypoint(60,166,18,40));
        sWaypoints.add(new Waypoint(75,116,30,40));
        sWaypoints.add(new Waypoint(120,116,0,40));

    }

    @Override
    public Path buildPath()
    {
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public List<Waypoint> getWaypoints()
    {
        return sWaypoints;
    }

    @Override
    public RigidTransform2d getStartPose()
    {
        return new RigidTransform2d(new Translation2d(18, 166), Rotation2d.fromDegrees(90.0));
    }

    @Override
    public boolean isReversed()
    {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":18,"y":166},"speed":0,"radius":0,"comment":""},{"position":{"x":60,"y":166},"speed":40,"radius":18,"comment":""},{"position":{"x":75,"y":116},"speed":40,"radius":30,"comment":""},{"position":{"x":120,"y":116},"speed":40,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: DriveToFarSwitchFromBPath
}