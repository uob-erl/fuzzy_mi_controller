function sysCall_init()
    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objName=sim.getObjectName(modelHandle)

    -- Check if the required plugin is there:
    local moduleName=0
    local moduleVersion=0
    local index=0
    local pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end

    camPub=simROS.advertise('/robot_cam', 'sensor_msgs/Image')
    simROS.publisherTreatUInt8ArrayAsString(camPub) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)
end

function sysCall_cleanup()

end

function sysCall_sensing()
    local data,w,h=sim.getVisionSensorCharImage(modelHandle)
    sim.transformImage(data,{w,h},4)
    d={}
    d['header']={seq=0,stamp=simROS.getTime(), frame_id="/robot_cam"}
    d['height']=h
    d['width']=w
    d['encoding']='rgb8'
    d['is_bigendian']=1
    d['step']=w*3
    d['data']=data
    simROS.publish(camPub,d)
end
