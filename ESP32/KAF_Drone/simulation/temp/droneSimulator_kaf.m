%% Full State Drone Simulator Environment
% Author: Kent Fukuda
% Last Updated: 10/25/2025
% Version: #$#>>0.2<<#$#
% For drone states, the indexes corresponds as follows:
% 1:3 position, 4:6 velocity, 7:9 attitude 10:12 attitude rate

classdef droneSimulator_kaf < handle
    properties
        drone; sim; ctrl;
    end
    methods
        function R = rotation( this, axial )
            theta = sqrt( sum( axial.^ 2 ) );
            cosT = cos( theta );
            cosT1 = 1 - cosT;
            sinT = sin( theta );
            if theta == 0
                theta = 1;
            end
            x = axial( 1 ) / theta;
            y = axial( 2 ) / theta;
            z = axial( 3 ) / theta;
            R = [ x * x * cosT1 +     cosT, x * y * cosT1 - z * sinT, x * z * cosT1 + y * sinT;
                  x * y * cosT1 + z * sinT, y * y * cosT1 +     cosT, y * z * cosT1 - x * sinT;
                  x * z * cosT1 - y * sinT, y * z * cosT1 + x * sinT, z * z * cosT1 +     cosT; ];
        end
        
        function euler = axial2euler( this, axial )
            theta = sqrt( sum( axial .^ 2 ) );
            unitvec = axial / theta;
            quat = sin( theta / 2 ) * [ cot( theta / 2 ); unitvec ];
            e0 = quat( 1 );
            ex = quat( 2 );
            ey = quat( 3 );
            ez = quat( 4 );
            euler = [
                atan2( 2 * ( e0 * ex + ey * ez ), ( e0 ^2 + ez ^2 - ex ^2 - ey ^2 ) );
                asin( 2 * ( e0 * ey - ex * ez ) );
                atan2( 2 * ( e0 * ez + ex * ey ), ( e0 ^2 + ex ^2 - ey ^2 - ez ^2 ) ) ] * 180 / pi;
            euler( isnan( euler ) ) = 0;
        end

        function axial = euler2axial( this, euler )
            sinphi = sin( euler( 1 ) / 2 * pi / 180 );
            cosphi = cos( euler( 1 ) / 2 * pi / 180 );
            sinpsi = sin( euler( 2 ) / 2 * pi / 180 );
            cospsi = cos( euler( 2 ) / 2 * pi / 180 );
            sintht = sin( euler( 3 ) / 2 * pi / 180 );
            costht = cos( euler( 3 ) / 2 * pi / 180 );
            e0 = cospsi * costht * cosphi + sinpsi * sintht * sinphi;
            ex = cospsi * costht * sinphi - sinpsi * sintht * cosphi;
            ey = sinpsi * costht * cosphi + cospsi * sintht * sinphi;
            ez = cospsi * sintht * cosphi - sinpsi * costht * sinphi;
            theta = 2 * acos( e0 );
            unitvec = [ ex; ey; ez ] / sin( theta / 2 );
            axial = theta * unitvec;
        end

        function [ timeRise, timeSettle, finalError ] = printSim( this, writeFilePath, verification )
            % calculate performance
            saveGraphs = exist( "writeFilePath", "var" );
            filter = this.ctrl.trigger == 1;
            actual = this.sim.trajresults( 1:3, filter );
            desired = this.sim.trajresults( 4:6, filter );
            maxT = this.ctrl.dt * ( size( this.ctrl.trigger, 2 ) - 1 );
            time = 0:this.ctrl.dt:maxT;
            time = time( filter );
            differences = sqrt( sum( ( actual - desired ) .^ 2 ) );% ./ sqrt( sum( desired .^ 2 ) );
            lastSecDiffs = differences( time > maxT - 1 );
            finalError = mean( lastSecDiffs );
            if max( lastSecDiffs ) - min( lastSecDiffs ) > 0.04
                timeSettle = inf;
            else
                timeSettle = max( time( abs( differences - finalError ) >= 0.02 ) );
            end
            timeRise = min( time( differences < finalError * 0.95 + 0.05 ) ) ...
                - max( time( differences > finalError * 0.05 + 0.95 ) );
            % plot difference
            figure;
            subplot( 1, 2, 1 );
            plot( time, differences, "-", "Color", "black" );
            hold on;
            if exist( "verification", "var" )
                interpVeri = zeros( size( time, 2 ), 3 );
                for i = 1:3
                    interpVeri( :, i ) = interp1( verification( :, 1 ), verification( :, i + 1 ), time );
                end
                plot( time, sqrt( sum( ( interpVeri' - desired ) .^ 2 ) ), "-", "Color", "blue" );
                legend( [ "Simulation", "Physical" ] );
            end
            % for i = 1:3
            %     plot( time(1), differences(1), ".", "Color", "white" );
            % end
            title( "Path Difference" );
            xlabel( "Time (s)" );
            ylabel( "|\chi_{target}-\chi_{actual}| (m)" );%|\chi_{target}|^{-1}
            xlim( [ 0, inf ] );
            ylim( [ 0, inf ] );
            % legend( [ "", append( "Rise Time T_r: ", num2str( timeRise ), "s" ), ...
            %         append( "Settling Time T_s: ", num2str( timeSettle ), "s" ), ...
            %         append( "Final Error E_{ss}: ", num2str( finalError * 100 ), "%" ) ], "Location", "southwest" );
            % plot trajectory
            subplot( 1, 2, 2 );
            view( 3 );
            plot3( desired( 1, : ), desired( 2, : ), desired( 3, : ), "-", "Color", "red" );
            hold on;
            plot3( actual( 1, : ), actual( 2, : ), actual( 3, : ), "-", "Color", "black" );
            legTable = [ "Target Path", "Actual Path" ];
            if exist( "verification", "var" )
                plot3( verification( :, 2 ), verification( :, 3 ), verification( :, 4 ), "-", "Color", "blue" );
                legTable = [ legTable, "Real-World Path" ];
            end
            title( "Drone Trajectory" );
            xlabel( "X-Position (m)" );
            ylabel( "Y-Position (m)" );
            zlabel( "Z-Position (m)" );
            axis equal;
            legend( legTable, "Location", "southeast" );
            if saveGraphs
                set( gcf, "Position", [ 1, 41, 1280, 607 ] );
                saveas( gcf, append( writeFilePath, "trajectory.fig" ) );
            end
            % plot motor output
            figure;
            outLen = size( this.ctrl.outputs, 1 );
            outLenSq = ceil( sqrt( outLen ) );
            legTable = string( outLen );
            colorTable = [ "black", "blue", "red", "green", "magenta", "cyan" ];
            plotIds = 1:( outLenSq ^ 2 * 2 );
            plotFilter = mod( plotIds - 1, outLenSq * 2 ) < outLenSq;
            subplot( outLenSq, 2 * outLenSq, plotIds( plotFilter ) );
            for i = 1:outLen
                plot( time, this.ctrl.outputs( i, filter ) * 100, "-", "Color", colorTable( i ) );
                legTable( i ) = append( "Motor ", num2str( i ) );
                hold on;
            end
            title( "Motor Outputs" );
            xlabel( "Time (s)" );
            ylabel( "Capacity (%)" );
            xlim( [ 0, inf ] );
            ylim( [ 0, 100 ] );
            legend( legTable, "Location", "northeast" );
            plotIds = plotIds( not( plotFilter ) );
            for i = 1:outLen
                subplot( outLenSq, 2 * outLenSq, plotIds( i ) );
                plot( time, this.ctrl.outputs( i, filter ) * 100, "-", "Color", colorTable( i ) );
                title( append( "Motor Output ", num2str( i ) ) );
                xlabel( "Time (s)" );
                ylabel( "Capacity (%)" );
                xlim( [ 0, inf ] );
                ylim( [ 0, 100 ] );
            end
            if saveGraphs
                set( gcf, "Position", [ 1, 41, 1280, 607 ] );
                saveas( gcf, append( writeFilePath, "motors.png" ) );
            end
            % plot states
            figure;
            titleTable = [ "Position x (m)", "Position y (m)", "Position z (m)", ...
                            "Velocity v_x (m/s)", "Velocity v_y (m/s)", "Velocity v_z (m/s)", ...
                         "Attitude \theta_x (rad)", "Attitude \theta_y (rad)", "Attitude \theta_z (rad)", ...
                         "Attitude Rate \omega_x (rad/s)", "Attitude Rate \omega_y (rad/s)", "Attitude Rate \omega_z (rad/s)" ];
            for i = 1:12
                subplot( 4, 3, i );
                plot( time, this.sim.actualtrajectory( i, filter ), "-", "Color", "black" );
                hold on;
                plot( time, this.sim.desiredtrajectory( i, filter ), "-", "Color", "red" );
                if exist( "verification", "var" ) &&  i <= 9
                    plot( verification( :, 1 ), verification( :, i + 1 ), "-", "Color", "blue" );
                end
                xlim( [ 0, time( end ) ] );
                xlabel( "Time (s)" );
                ylabel( titleTable( i ) );
            end
            if saveGraphs
                set( gcf, "Position", [ 1, 41, 1280, 607 ] );
                saveas( gcf, append( writeFilePath, "state.png" ) );
            end
        end

        function [ stateStep ] = stepSim( this, time, state )
            % control step
            controlIndex = floor( time / this.ctrl.dt ) + 1;
            if this.ctrl.trigger( controlIndex ) == 0
                this.ctrl.idx = controlIndex;
                this.ctrl.input = this.sim.trajFcn( time );
                this.ctrl.trigger( controlIndex ) = 1;
                this.ctrl.desired = state;
                for i = 1:size( this.drone.s, 2 )
                    if time > this.drone.s( i ).lastTimerCall
                        this.drone.s( i ).loop( state, this, i );
                        this.drone.s( i ).lastTimerCall = time;
                    end
                end
                this.ctrl.outputs( :, controlIndex ) = this.ctrl.output;
                this.sim.actualtrajectory( :, controlIndex ) = state;
                this.sim.desiredtrajectory( :, controlIndex ) = this.ctrl.desired;
                this.sim.trajresults( 1:3, this.ctrl.idx ) = state( 1:3 );
                this.sim.trajresults( 4:6, this.ctrl.idx ) = this.ctrl.desired( 1:3 );
            end
            % physics step
            disturbance = this.sim.distFcn( time, state );
            ttau = this.drone.M * ( this.drone.w .* this.ctrl.outputs( :, controlIndex ) ) .^ 2;
            R = rotation( this, state( 7:9 ) );
            wxw = -cross( state( 10:12 ), this.drone.J * state( 10:12 ) );
            stateStep = zeros( 12, 1 );
            stateStep( 10:12 ) = this.drone.i.J * ( wxw + ttau( 2:4 ) ) + disturbance( 4:6 );
            stateStep( 7:9 ) = R * state( 10:12 );
            stateStep( 4:6 ) = R * [ 0; 0; ttau( 1 ) ] / this.drone.m + disturbance( 1:3 );
            stateStep( 1:3 ) = state( 4:6 );
        end

        function [ time, states ] = runSim( this, args, Tf, Y0, trajFcn, distFcn, initOnly )
            if ~exist( "Y0", "var" )
                Y0 = zeros( 12, 1 );
            end
            if ~exist( "trajFcn", "var" )
                trajFcn = @( t ) [ 0; 0; 0 ];
            end
            if ~exist( "distFcn", "var" )
                distFcn = @( t, y ) [ 0; 0; -9.8; 0; 0; 0 ];
            end
            if ~exist( "initOnly", "var" )
                initOnly = false;
            end
            this.sim.trajFcn = trajFcn;
            this.sim.distFcn = distFcn;
            this.ctrl.dt = 100;
            this.drone.record = [];
            for i = 1:size( this.drone.s, 2 )
                this.drone.s( i ).init( args, this, i );
                this.drone.s( i ).lastTimerCall = 0;
                this.ctrl.dt = min( this.ctrl.dt, this.drone.s( i ).dt );
            end
            recordCount = floor( Tf / this.ctrl.dt ) + 1;
            this.ctrl.output = zeros( size( this.drone.w, 1 ), 1 );
            this.ctrl.trigger = zeros( 1, recordCount );
            this.ctrl.outputs = nan( size( this.drone.w, 1 ), recordCount );
            this.sim.actualtrajectory = nan( 12, recordCount );
            this.sim.desiredtrajectory = nan( 12, recordCount );
            this.sim.trajresults = nan( 6, recordCount );
            if ~initOnly
                [ time, states ] = ode45( @( t, y ) this.stepSim( t, y ), [ 0, Tf ], Y0, odeset( MaxStep = this.ctrl.dt / 5 ) );
            end
        end

        function this = droneSimulator_kaf( writeFilePath, readFilePaths )
            hasOutFile = exist( "writeFilePath", "var" );
            if ~exist( "readFilePaths", "var" )
                readFilePaths = "drone.mat";
                if hasOutFile
                    readFilePaths = append( writeFilePath, readFilePaths );
                end
                if ~exist( readFilePaths, "file" )
                    readFilePaths = "Drone Body.xlsx";
                else
                    hasOutFile = false;
                end
            end
            for readFilePath = readFilePaths
                if ~endsWith( readFilePath, ".mat" )
                    droneTable = readtable( readFilePath, "Sheet", "Drone Parts" );
                    simTable = readtable( readFilePath, "Sheet", "Simulation Functions" );
                    droneTable.mass_kg = droneTable.mass_N / 9.8;
                    massTable = droneTable( droneTable.mass_include == 1, : );
                    drone.m = sum( massTable.mass_kg );
                    com = sum( massTable.mass_kg .* [ massTable.x_CM, massTable.x_CM, massTable.x_CM ] ) / drone.m;
                    drone.J = zeros( 3, 3 );
                    droneTable.x_M = ( droneTable.x_CM - com( :, 1 ) ) / 100;
                    droneTable.y_M = ( droneTable.y_CM - com( :, 2 ) ) / 100;
                    droneTable.z_M = ( droneTable.z_CM - com( :, 3 ) ) / 100;
                    droneTable.sx_M = droneTable.sx_CM / 100;
                    droneTable.sy_M = droneTable.sy_CM / 100;
                    droneTable.sz_M = droneTable.sz_CM / 100;
                    massTable = droneTable( droneTable.mass_include == 1, : );
                    if hasOutFile
                        figure;
                        view( 3 );
                    end
                    for i = 1:size( massTable, 1 )
                        x = massTable.x_M( i );
                        y = massTable.y_M( i );
                        z = massTable.z_M( i );
                        sx = massTable.sx_M( i );
                        sy = massTable.sy_M( i );
                        sz = massTable.sz_M( i );
                        drone.J = drone.J + massTable.mass_kg( i )* ( ...
                            [ sy * sy + sz * sz, 0, 0; 0, sx * sx + sz * sz, 0; 0, 0, sx * sx + sy * sy ] / 12 + ...
                            [ y * y + z * z,    -x * y,        -x * z;     ...
                                 -x * y,     x * x + z * z,    -y * z;     ...
                                 -x * z,        -y * z,     x * x + y * y ] );
                        xyz = [ x - sx, x + sx, x + sx, x - sx, x - sx, x + sx, x + sx, x - sx;
                                y - sy, y - sy, y + sy, y + sy, y - sy, y - sy, y + sy, y + sy;
                                z - sz, z - sz, z - sz, z - sz, z + sz, z + sz, z + sz, z + sz ]';
                        idx = [ 4 8 5 1 4; 1 5 6 2 1; 5 8 7 6 5; 1 4 3 2 1; 2 6 7 3 2; 3 7 8 4 3; ]';
                        if hasOutFile
                            text( x, y, z, massTable.part( i ), "Color", "blue" );
                            patch( xyz( idx, 1 ), xyz( idx, 2 ), xyz( idx, 3 ), "red", "facealpha", 0 );
                            hold on;
                        end
                    end
                    if hasOutFile
                        axis equal;
                        xlabel( "x (m)" );
                        ylabel( "y (m)" );
                        zlabel( "z (m)" );
                        title( "Drone Frame " );
                        saveas( gcf, append( writeFilePath, "drone.fig" ) );
                    end
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
                    drone.w = thrustTable.max_motor_RADS;
                    drone.M = [ cTrhoD44pi'; lowerM ] + [ zeros( 3, size( cTrhoD44pi, 1 ) ); cQrhoD54pi' ];
                    drone.s = [];
                    simTable = simTable( simTable.include == 1, : );
                    for i = 1:size( simTable, 1 )
                        o.type = simTable.type( i );
                        if ~isempty( simTable.init_function{ i } )
                            o.init = eval( append( "@(args,cfg,id)", simTable.init_function( i ), "(args,cfg,id)" ) );
                        else
                            o.init = @( args, cfg, id ) 0;
                        end
                        if ~isempty( simTable.loop_function{ i } )
                            o.loop = eval( append( "@(state,cfg,id)", simTable.loop_function( i ), "(state,cfg,id)" ) );
                        else
                            o.loop = @( cfg ) cfg;
                        end
                        o.dt = 1 / simTable.loop_rate_HZ( i );
                        drone.s = [ drone.s, o ];
                    end
                    drone.i.M = inv( drone.M );
                    drone.i.J = inv( drone.J );
                    drone.explanation.m = "d.m is the mass of the entire drone in kg";
                    drone.explanation.I = "d.I is the moment of inertia matrix given in kgm^2 from drone center of mass";
                    drone.explanation.M = "d.M is the mixing matrix converting propeller rotation in rad/s^2 into torque in Nm";
                    drone.explanation.w = "d.w is the maximum propeller angular velocity in rad/s";
                else
                    load( readFilePath, "drone" );
                end
                if hasOutFile
                    save( append( writeFilePath, "drone.mat" ), "drone" );
                end
                this.drone = [ this.drone, drone ];
            end
            this.sim = [];
            this.ctrl = [];
        end
    end
end