classdef drone_utils
    properties
        drones; times; ttaus;
    end
    methods
        function this = drone_utils( drones )
            this.drones = drones;
            this.times = zeros( length( drones ), 1 );
            this.ttaus = zeros( 4, length( drones ) );
        end
    end
    methods( Static )
        function R = rotation( quaternion )
            r = quaternion( 1 );
            x = quaternion( 2 );
            y = quaternion( 3 );
            z = quaternion( 4 );
            R = 2 * [ 0.5 - y * y - z * z, x * y - r * z, x * z + r * y; 
                      x * y + r * z, 0.5 - x * x - z * z, y * z - r * x; 
                      x * z - r * y, y * z + r * x, 0.5 - x * x - y * y ];
        end
        function dx = step_flight( utils, ad, ts, t, x, logger )
            dx = zeros( size( x ) );
            for i = 1:length( utils.drones )
                idx = ( ( i - 1 ) * 13 + 1 ):( i * 13 );
                xi = x( idx );
                if utils.times( i ) + ts <= t
                    [ motors, logdata ] = utils.drones( i ).code.flight( t - utils.times( i ), xi );
                    utils.ttaus( :, i ) = utils.drones( i ).phys.M * ( utils.drones( i ).phys.w .* motors ) .^ 2;
                    utils.times( i ) = t;
                    if exist( "logger", "var" )
                        log.id = utils.drones.code.id;
                        log.logdata = logdata;
                        log.motors = motors;
                        log.x = xi;
                        logger( "step_flight", t, log );
                    end
                elseif utils.time > t
                    time_backwards_error;
                end
                R = drone_utils.rotation( xi( 7:10 ) );
                W = [ 0, -xi( 11 ), -xi( 12 ), -xi( 13 ); xi( 11 ), 0, xi( 13 ), -xi( 12 ); 
                      xi( 12 ), -xi( 13 ), 0, xi( 11 ); xi( 13 ), xi( 12 ), -xi( 11 ), 0 ];
                wxw = -cross( xi( 11:13 ), utils.drones( i ).phys.J * xi( 11:13 ) );
                dx( idx ) = [ xi( 4:6 );
                              R * [ 0; 0; utils.ttaus( 1, i ) ] / utils.drones( i ).phys.m + ad( 1:3 );
                              0.5 * W * xi( 7:10 );
                              utils.drones( i ).invt.J * ( wxw + utils.ttaus( 2:4, i ) ) + ad( 4:6 ) ];
            end
        end

        function p = step_coms( drones, t, p, logger )
            if isempty( p )
                packet = [];
            else
                packet = p{ 1 };
                p = p( 2:end );
                if exist( "logger", "var" )
                    log.to = char( packet( 1 ) );
                    log.from = char( packet( 2 ) );
                    log.type = dec2hex( packet( 3 ) );
                    log.id = dec2hex( packet( 4 ) );
                    logger( "step_coms", t, log );
                end
            end
            for drone = drones
                p = [ p, drone.code.comms( t, packet ) ];
            end
        end
        function [ motors, Rs, xs ] = step_sensor( drones, dt, dv, w, q )
            motors = [];
            Rs = [];
            xs = [];
            for i = 1:length( drones )
                R = drone_utils.rotation( q );
                accel = R * ( dv + [ 0; 0; 9.81 ] ) + drones( i ).sens.ofst( :, 1 ) + ...
                        randn( 3, 1 ) .* drones( i ).sens.stdv( :, 1 );
                gyro = w + drones( i ).sens.ofst( :, 2 ) + randn( 3, 1 ) .* drones( i ).sens.stdv( :, 2 );
                mag = R * [ 1; 0; 0 ] + drones( i ).sens.ofst( :, 3 ) + randn( 3, 1 ) .* drones( i ).sens.stdv( :, 3 );
                [ motor, R, estimate ] = drones( i ).code.flight( dt, [ accel', gyro', mag' ], drones( i ).sens.mask );
                motors = [ motors, motor ];
                Rs = [ Rs, R ];
                xs = [ xs, estimate ];
            end
        end
        function drones = init_drones( drones )
            for i = 1:length( drones )
                drones( i ).code.init();
                drones( i ).data = [];
            end
            for i = 1:length( drones )
                drone_utils.step_coms( drones( i ), 8001, { [ char( drones( i ).code.id ), 'G', bitor( 64, 12 ), 0, 2 ] } );
            end
            for t = 0:1:1000
                drone_utils.step_sensor( drones, 0.001, [ 0; 0; 0 ], [ 0; 0; 0 ], [ 1; 0; 0; 0 ] );
            end
            for i = 1:length( drones )
                drone_utils.step_coms( drones( i ), 8002, { [ char( drones( i ).code.id ), 'G', bitor( 64, 12 ), 0, 5 ] } );
            end
        end
        function [ drones, y ] = assemble_swarm( ids, loadPath )
            drones = [];
            y = zeros( length( ids ) * 13, 1 );
            for id = ids
                load( fullfile( loadPath, append( id, "_drone.mat" ) ), "drone" );
                drones = [ drones, drone ];
            end
        end
        function drone = compile_drone( deviceID, loadPath, savePath )
            codeName = append( deviceID, "_code" );
            mex( "-output", fullfile( savePath, codeName ), "drone_bridge.cpp", "../core/*.cpp" );
            drone.code.id = deviceID;
            drone.code.init = eval( append( "@()", codeName, "('R',uint8('", deviceID, "'))" ) );
            drone.code.comms = eval( append( "@(t,data)", codeName, "('C',t,uint8(data))" ) );
            drone.code.flight = eval( append( "@(dt,data,mask)", codeName, "('F',dt,data,mask)" ) );
            drone.sens.ofst = randn( 3, 3 ) .* [ 0.2, 0.2, 0.2; 0.03, 0.03, 0.03; 1, 1, 1 ]';
            drone.sens.stdv = randn( 3, 3 ) .* [ 0.4, 0.4, 0.4; 0.12, 0.12, 0.12; 1, 1, 1 ]';
            drone.sens.mask = 'TTF';
            drone.data = [];
            droneTable = readtable( loadPath );
            droneTable.mass_kg = droneTable.mass_N / 9.8;
            massTable = droneTable( droneTable.mass_include == 1, : );
            drone.phys.m = sum( massTable.mass_kg );
            com = sum( massTable.mass_kg .* [ massTable.x_CM, massTable.x_CM, massTable.x_CM ] ) / drone.phys.m;
            drone.phys.J = zeros( 3, 3 );
            droneTable.x_M = ( droneTable.x_CM - com( :, 1 ) ) / 100;
            droneTable.y_M = ( droneTable.y_CM - com( :, 2 ) ) / 100;
            droneTable.z_M = ( droneTable.z_CM - com( :, 3 ) ) / 100;
            droneTable.sx_M = droneTable.sx_CM / 100;
            droneTable.sy_M = droneTable.sy_CM / 100;
            droneTable.sz_M = droneTable.sz_CM / 100;
            massTable = droneTable( droneTable.mass_include == 1, : );
            figure;
            view( 3 );
            for i = 1:size( massTable, 1 )
                x = massTable.x_M( i );
                y = massTable.y_M( i );
                z = massTable.z_M( i );
                sx = massTable.sx_M( i );
                sy = massTable.sy_M( i );
                sz = massTable.sz_M( i );
                drone.phys.J = drone.phys.J + massTable.mass_kg( i )* ( ...
                    [ sy * sy + sz * sz, 0, 0; 0, sx * sx + sz * sz, 0; 0, 0, sx * sx + sy * sy ] / 12 + ...
                    [ y * y + z * z,    -x * y,        -x * z;     ...
                         -x * y,     x * x + z * z,    -y * z;     ...
                         -x * z,        -y * z,     x * x + y * y ] );
                xyz = [ x - sx, x + sx, x + sx, x - sx, x - sx, x + sx, x + sx, x - sx;
                        y - sy, y - sy, y + sy, y + sy, y - sy, y - sy, y + sy, y + sy;
                        z - sz, z - sz, z - sz, z - sz, z + sz, z + sz, z + sz, z + sz ]';
                idx = [ 4 8 5 1 4; 1 5 6 2 1; 5 8 7 6 5; 1 4 3 2 1; 2 6 7 3 2; 3 7 8 4 3; ]';
                text( x, y, z, massTable.part( i ), "Color", "blue" );
                patch( xyz( idx, 1 ), xyz( idx, 2 ), xyz( idx, 3 ), "red", "facealpha", 0 );
                hold on;
            end
            axis equal;
            xlabel( "x (m)" );
            ylabel( "y (m)" );
            zlabel( "z (m)" );
            title( "Drone Frame " );
            saveas( gcf, fullfile( savePath, append( deviceID, "_fig.fig" ) ) );
            thrustTable = droneTable( droneTable.thrust_include == 1, : );
            thrustTable.max_motor_RADS = thrustTable.max_motor_RPM / 60 * 2 * pi;
            cTrhoD44pi = thrustTable.max_thrust_N ./ thrustTable.max_motor_RADS .^ 2;
            cQrhoD54pi = thrustTable.max_power_W ./ thrustTable.max_motor_RADS .^ 3;
            lowerM = zeros( 3, size( cTrhoD44pi, 1 ) );
            for i = 1:size( cTrhoD44pi, 1 )
                x = thrustTable.x_M( i );
                y = thrustTable.y_M( i );
                z = thrustTable.z_M( i );
                lowerM( :, i ) = cTrhoD44pi( i ) * cross( [ x, y, z ], [ 0, 0, 1 ] )';
            end
            drone.phys.w = thrustTable.max_motor_RADS;
            drone.phys.M = [ cTrhoD44pi'; lowerM ] + [ zeros( 3, size( cTrhoD44pi, 1 ) ); cQrhoD54pi' ];
            drone.invt.m = 1 / drone.phys.m;
            drone.invt.M = inv( drone.phys.M );
            drone.invt.w = 1 / drone.phys.w;
            drone.invt.J = inv( drone.phys.J );
            drone.info.m = ".m is the mass of the entire drone in kg";
            drone.info.I = ".I is the moment of inertia matrix given in kgm^2 from drone center of mass";
            drone.info.M = ".M is the mixing matrix converting propeller rotation in rad/s^2 into torque in Nm";
            drone.info.w = ".w is the maximum propeller angular velocity in rad/s";
            save( fullfile( savePath, append( deviceID, "_drone.mat" ) ), "drone" );
        end
    end
end