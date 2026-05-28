clear; clc; close all;
drone = create_drone( 'A', "sim", "compile" );
drone.init();
drone.comms( 8001, [ 'A', 'G', bitor( 64, 30 ), 0, 1 ] );
drone.calib();
drone.calib();
%%
doCalib( drone, 1000,  6,  7, 13, false );
doCalib( drone, 1000,  3,  4, 30, false );
%%
doCalib( drone, 1000, 13, 13, 10, true );
%%
doCalib( drone, 100, 13, 13, 1, true );

function doCalib( drone, len, id1, id2, itr, graph )
    for i = 1:itr
        drone.calib();
        time = 0:0.01:10;
        value = zeros( len, 1 );
        estimate = zeros( len, 1 );
        for j = 1:len
            state = drone.calib();
            value( j ) = state( id1 );
            stateest = drone.comms( 8001, [ 'A', 'G', bitor( 64, 4 ), 0, 1 ] );
            stateest = typecast( stateest{1}, "single" );
            estimate( j ) = stateest( id2 );
        end
        if graph
            figure;
            plot( time( 1:len ), value, "-", "Color", "black" );
            hold on;
            plot( time( 1:len ), estimate, "-", "Color", "red" );
            xlabel( "Time (s)" );
        end
    end
 end