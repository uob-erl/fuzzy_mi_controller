
-- Takes cmd_vel from ROS, impliments differential drive kinematics and sends commands to the motors
function callback_MotorsVelocity(msg)

    cmd_velX = msg.linear.x
    cmd_velW = msg.angular.z
    
end


-- Tf transform message
function getTransformMsg(robotHandle,name,relTo,relToName)

    local pose = sim.getObjectPosition(robotHandle,-1)
    local rotation = sim.getObjectQuaternion(robotHandle,-1)

    return {
        header={
            stamp = simGetSimulationTime(),
            frame_id = "odom"
        },
        child_frame_id = "base_link",
        transform={
            translation={x=pose[1],y=pose[2],z=pose[3]},
            rotation={x=rotation[1],y=rotation[2],z=rotation[3],w=rotation[4]}
        }
    }
end

-- Odometry message
function getOdometryMsg(robotHandle,name,relTo,relToName)

    local pose = sim.getObjectPosition(robotHandle,relTo)
    local orientation = sim.getObjectQuaternion(robotHandle,relTo)
    local v_linear, v_angular = sim.getObjectVelocity(robotHandle)
    local dt=simGetSimulationTimeStep()
    local v = (v_linear[1]^2 + v_linear[2]^2 + v_linear[3]^2)^0.5

    return {
        header={
            stamp=simGetSimulationTime(),
            frame_id=relToName
        },
        child_frame_id=name,
        pose={
            pose={
              position={x=pose[1],y=pose[2],z=pose[3]},
              orientation={x=orientation[1],y=orientation[2],z=orientation[3],w=orientation[4]}
            },
        },
        twist={
            twist={
              linear={x=v, y=v0, z=0},
              angular={x=v_angular[1],y=v_angular[2],z=v_angular[3]}
            },
        },
    }
end

-- initialization
function sysCall_init()
    -- Check if the RosInterface is available:
    moduleName=0
    moduleVersion=0
    index=0
    pluginFound=false

    L = 0.36 -- lenght between the two wheels
    R = 0.0975 -- wheel radius
    cmd_velW=0
    cmd_velX=0
    Vr=0
    Vl=0

    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginFound=true
        end
        index=index+1
    end

    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)

    leftMotor=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    rightMotor=sim.getObjectHandle("Pioneer_p3dx_rightMotor")

    -- Wheel distance
    --lwheel = sim.getObjectPosition(sim.getObjectHandle("Pioneer_p3dx_leftWheel"),robotHandle)
    --rwheel = sim.getObjectPosition(sim.getObjectHandle("Pioneer_p3dx_rightWheel"),robotHandle)
    --distWheels = math.abs(lwheel[2]) + math.abs(rwheel[2])
    --print("Wheel distance",distWheels)

    if pluginFound then
        cmd_velSub=simROS.subscribe('/cmd_vel', 'geometry_msgs/Twist', 'callback_MotorsVelocity')
        odomPub=simROS.advertise('/odom', 'nav_msgs/Odometry')
        clockPub=simROS.advertise('/clock','rosgraph_msgs/Clock')

        -- simStepDonePub=simROS.advertise('/simulationStepDone', 'std_msgs/Bool')
        -- simStatePub=simROS.advertise('/simulationState','std_msgs/Int32')
        -- simTimePub=simROS.advertise('/simulationTime','std_msgs/Float32')
        -- auxPub=simROS.advertise('/privateMsgAux', 'std_msgs/Bool')
        -- auxSub=simROS.subscribe('/privateMsgAux', 'std_msgs/Bool', 'aux_callback')
        -- rosInterfaceSynModeEnabled=false

    else
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    collider = sim.getCollisionHandle("envCollisions")
    collCount=0
    prevCollisionTime=sim.getSystemTimeInMs(-1)


end


function sysCall_cleanup()

end

-- atm collision sensing
function sysCall_sensing()

  -- counting collisions
    res,_ = sim.handleCollision(collider)

    if (res==1 and sim.getSystemTimeInMs(prevCollisionTime) > 2000) then
        prevCollisionTime=sim.getSystemTimeInMs(-1)
        collCount=collCount+1
        print(collCount)
    end
end

    -- Perform robot control tasks here
function sysCall_actuation()

     -- calculating velocities and sending motor commands
     Vr = (cmd_velX + (L/2) * cmd_velW) * 10
     Vl = (cmd_velX - (L/2) * cmd_velW) * 10

     sim.setJointTargetVelocity(leftMotor,Vl)
     sim.setJointTargetVelocity(rightMotor,Vr)

    -- Publish tf transform and odometry; -1 in V-rep refers to world (odom) frame
    simROS.sendTransform(getTransformMsg(robotHandle,'/base_link',-1,'/odom'))
    simROS.publish(odomPub,getOdometryMsg(robotHandle,'/base_link',-1,'/odom'))

    simROS.publish(clockPub,{clock=simGetSimulationTime()}) -- the /clock sim time topic in ROS

end
