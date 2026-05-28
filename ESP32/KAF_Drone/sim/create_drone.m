function drone = create_drone( deviceID, type, arg )
    drone.id = deviceID;
    if strcmp( type, "sim" )
        codeName = append( deviceID, "_code" );
        if strcmp( arg, "compile" )
            mex( "-output", codeName, "../src/kaf_quadcopter_code.cpp", ...
                    "../src/core/*.cpp", "../src/auxilary/commander.cpp" , "../src/auxilary/pid_tuner.cpp" );
        end
        drone.init = eval( append( "@()", codeName, "('R',uint8('", deviceID, "'))" ) );
        drone.comms = eval( append( "@(t,data)", codeName, "('C',t,uint8(data))" ) );
        drone.flight = eval( append( "@(dt,data,mask)", codeName, "('F',dt,data,mask)" ) );
        drone.command = eval( append( "@(t)", codeName, "('E',t)" ) );
        drone.calib = eval( append( "@()", codeName, "('S')" ) );
    elseif strcmp( type, "serial" )
        drone.fcn = serialport( arg, 115200 );
        pause( 2 );
        if drone.fcn.NumBytesAvailable > 0
            disp( char( drone.fcn.read( drone.fcn.NumBytesAvailable, "uint8" ) ) )
        end
    end
end