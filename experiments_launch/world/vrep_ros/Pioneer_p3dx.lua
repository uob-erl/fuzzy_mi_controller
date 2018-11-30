function cmdVel_callback(msg)

end

function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(leftMotor,msg.data)
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(rightMotor,msg.data)
end

function callback_MotorsVelocity(msg)
    V= msg.linear.x
    W= msg.angular.z
    L= distWheels
    R = 0.0925
    C = 2*math.pi*R

    Vr=V+(L/2)*W
    Vl=V-(L/2)*W

    -- Wr = Vr/R
    -- Wl = Vl/R

    Vl = Vl/C
    Vr = Vr/C
    Vl=Vl*2*math.pi
    Vr=Vr*2*math.pi

    simSetJointTargetVelocity(leftMotor,Vl)
    simSetJointTargetVelocity(rightMotor,Vr)
    simAddStatusbarMessage(string.format("Vl:%f Vr:%f",Vl, Vr))
end

-- Tf transform message
function getTransformMsg(objHandle,name,relTo,relToName,time)
    local p=sim.getObjectPosition(objHandle,relTo)
    local o=sim.getObjectQuaternion(objHandle,relTo)

    return {
        header={
            stamp=simGetSimulationTime(),
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

-- Odometry message
function getOdometryMsg(objHandle,name,relTo,relToName,time)
    local p=sim.getObjectPosition(objHandle,relTo)
    local o=sim.getObjectQuaternion(objHandle,relTo)
    print("Odom position:",p)
    print("Odom orientation:",o)
    local lv,av=sim.getObjectVelocity(objHandle)

    return {
        header={
            stamp=simGetSimulationTime(),
            frame_id=relToName
        },
        child_frame_id=name,
        pose={
            pose={
              position={x=p[1],y=p[2],z=p[3]},
              orientation={x=o[1],y=o[2],z=o[3],w=o[4]}
            },
        },
        twist={
            twist={
              linear={x=lv[1],y=lv[2],z=lv[3]},
              angular={x=av[1],y=av[2],z=av[3]}
            },
        },
    }
end

function sysCall_init()
    -- Check if the RosInterface is available:
    moduleName=0
    moduleVersion=0
    index=0
    pluginFound=false
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginFound=true
        end
        index=index+1
    end

    usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,16,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end
    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    leftMotor=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    rightMotor=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    noDetectionDist=0.5
    maxDetectionDist=0.1
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    v0=2

    -- Wheel distance
    lwheel = sim.getObjectPosition(sim.getObjectHandle("Pioneer_p3dx_leftWheel"),robotHandle)
    --rwheel = sim.getObjectPosition(sim.getObjectHandle("Pioneer_p3dx_rightWheel",robotHandle))
    distWheels = math.abs(lwheel[2]*2)
    print("Wheel distance",distWheels)

    if pluginFound then
        leftMotorSub=simROS.subscribe('/leftMotor','std_msgs/Float32','setLeftMotorVelocity_cb')
        rightMotorSub=simROS.subscribe('/rightMotor','std_msgs/Float32','setRightMotorVelocity_cb')
        cmd_velSub=simROS.subscribe('/cmd_vel', 'geometry_msgs/Twist', 'callback_MotorsVelocity')
        odomPub=simROS.advertise('/odom', 'nav_msgs/Odometry')
        clockPub=simROS.advertise('/clock','rosgraph_msgs/Clock')
        --tfPub=simROS.advertise('/tf', 'tf2_msgs/TFMessage')

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
-- This is a very simple EXAMPLE navigation program, which avoids obstacles using the Braitenberg algorithm


function sysCall_cleanup()

end

function sysCall_sensing()
    res,_ = sim.handleCollision(collider)

    if (res==1 and sim.getSystemTimeInMs(prevCollisionTime) > 1000) then
        prevCollisionTime=sim.getSystemTimeInMs(-1)
        collCount=collCount+1
        print(collCount)
    end
end

function sysCall_actuation()
    -- Perform robot control tasks here
    _,vlMotorSpd = sim.getObjectFloatParameter(leftMotor,sim.jointfloatparam_velocity)
    _,vrMotorSpd = sim.getObjectFloatParameter(rightMotor,sim.jointfloatparam_velocity)
    odomMotorSpd = (vlMotorSpd+vrMotorSpd)/20

    L= distWheels
    odomAngMotorSpd = ((vrMotorSpd-vlMotorSpd)/L)/10


    local t=simROS.getTime()
    --CURRENTLY PUBLISHED ON DIFFERENT TIMESTAMPS!!!!
    -- Publish tf transform; -1 in V-rep refers to world (odom) frame
    simROS.sendTransform(getTransformMsg(robotHandle,'/base_link',-1,'/odom'))
    -- Publish odometry; -1 in V-rep refers to world (odom) frame
    simROS.publish(odomPub,getOdometryMsg(robotHandle,'/base_link',-1,'/odom'))
    simROS.publish(clockPub,{clock=simGetSimulationTime()})




    -- Sample code
    -- for i=1,16,1 do
    --     res,dist=sim.readProximitySensor(usensors[i])
    --     if (res>0) and (dist<noDetectionDist) then
    --         if (dist<maxDetectionDist) then
    --             dist=maxDetectionDist
    --         end
    --         detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
    --     else
    --         detect[i]=0
    --     end
    -- end
    --
    -- vLeft=v0
    -- vRight=v0
    --
    -- for i=1,16,1 do
    --     vLeft=vLeft+braitenbergL[i]*detect[i]
    --     vRight=vRight+braitenbergR[i]*detect[i]
    -- end
    --
    -- sim.setJointTargetVelocity(leftMotor,vLeft)
    -- sim.setJointTargetVelocity(rightMotor,vRight)
end
