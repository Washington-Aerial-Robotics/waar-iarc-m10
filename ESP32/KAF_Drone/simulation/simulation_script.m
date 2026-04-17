%% Initialize Script
clear; clc; close all;
dronepath = "./drones/"; 
addpath( dronepath );
%% Compile Drones
drone_utils.compile_drone( "A", "./Crazyflie.xlsx", dronepath );
%drone_utils.compile_drone( "B", "./Crazyflie.xlsx", dronepath );
%drone_utils.compile_drone( "C", "./Crazyflie.xlsx", dronepath );
%drone_utils.compile_drone( "D", "./Crazyflie.xlsx", dronepath );
%% Setup Environment
[ drones, y ] = drone_utils.assemble_swarm( [ "A" ], dronepath );
%[ drones, y ] = drone_utils.assemble_swarm( [ "A", "B", "C", "D" ], dronepath );
drone_utils.init_drones( drones );
%% Attitude Estimation Test
attitude = [ 0.707; 0.707; 0; 0 ];%[ 0.9239; 0; 0; 0.3827 ];
coords = [ 0, 1, 1, 1, 0, -1, -1, -1, 0,  1,  1,  1, 0, -1, -1, -1, 0, 0, 0; 
           0, 1, 1, 1, 0,  1,  1,  1, 0, -1, -1, -1, 0, -1, -1, -1, 0, 0, 0; 
           0, 0, 1, 0, 0,  0,  1,  0, 0,  0,  1,  0, 0,  0,  1,  0, 0, 1, 0 ];
for i = 1:100
    [ motor, R, xs ] = drone_utils.step_sensor( drones( 1 ), 0.01, [ 0; 0; 0 ], [ 0; 0; 0 ], attitude );
end
coords( 3, [ 3, 7, 11, 15 ] ) = motor;
rotMatTrue = drone_utils.rotation( attitude ) * coords;
rotMatEstimate = drone_utils.rotation( [ cos( 0.5 * xs( 9 ) ); 0; 0; sin( 0.5 * xs( 9 ) ) ] ) * ...
    drone_utils.rotation( [ sqrt( 2 - sum( xs( 7:8 ) .^ 2 ) ); xs( 7:8 ); 0 ] / sqrt( 2 ) ) * coords;
rotMatOut = [ R( 1:3 )'; R( 4:6 )'; R( 7:9 )' ] * coords;
figure;
view( 3 );
plot3( rotMatTrue( 1, : ), rotMatTrue( 2, : ), rotMatTrue( 3, : ), ".-", "Color", "black" );
hold on;
plot3( rotMatEstimate( 1, : ), rotMatEstimate( 2, : ), rotMatEstimate( 3, : ), ".-", "Color", "red" );
hold on;
plot3( rotMatOut( 1, : ), rotMatOut( 2, : ), rotMatOut( 3, : ), ".-", "Color", "blue" );
xlim( [ -1.5, 1.5 ] );
ylim( [ -1.5, 1.5 ] );
zlim( [ -1.5, 1.5 ] );
legend( [ "True", "Estimate", "Output" ] );

%% Communication Test
% gsid = 'G';
% target = 'A';
% mid = 5;
% % Ping for Response
% data = drone_utils.step_coms( drones, 8001, { [ target, gsid, bitor( 0b01000000, 00 ), mid ] } );
% disp( append( "Success=", num2str( min( data{1} == [ gsid, target, 00, mid ] ) ) ) )
% % Flight Mode (Motors)
% data = drone_utils.step_coms( drones, 8001, { [ target, gsid, bitor( 0b01000000, 12 ), mid, 0b00000100 ] } );
% disp( append( "Success=", num2str( min( data{1} == [ gsid, target, 60, mid ] ) ) ) )
% % Set Setpoint
% setpoints = [ 3.2, 0.0, -1.2 ];
% data = drone_utils.step_coms( drones, 8001, { [ target, gsid, bitor( 0b01000000, 15 ), mid, ...
%       length( setpoints ), typecast( single( setpoints ), 'uint8' ) ] } );
% disp( append( "Success=", num2str( min( data{1} == [ gsid, target, 60, mid ] ) ) ) )
% % Set Motor Values
% motor = [ 0.9, 0.4, 0.3, 0.1 ];
% data = drone_utils.step_coms( drones, 8001, { [ target, gsid, bitor( 0b01000000, 16 ), mid, ...
%       typecast( single( motor ), 'uint8' ) ] } );
% disp( append( "Success=", num2str( min( data{1} == [ gsid, target, 60, mid ] ) ) ) )
% % Request WiFi IP Address
% data = drone_utils.step_coms( drones, 8001, { [ target, gsid, bitor( 0b01000000, 23 ), mid ] } );
% disp( append( "IP Address: ", char( data{ 1 }( 5:end ) ) ) )
% % Set WiFi Network and Password
% name = 'iPhone';
% pass = 'jtj35u02veac9';
% send = [ target, gsid, bitor( 64, 24 ), mid, name, zeros( 1, 20 - length( name ) ), pass, zeros( 1, 20 - length( pass ) ) ];
% disp( join( string( dec2hex( uint8( send ) ) )' ) )
% data = drone_utils.step_coms( drones, 8001, { send } );
% disp( append( "Success=", num2str( min( data{1} == [ gsid, target, 24, mid ] ) ) ) )
% % Kill Function
% data = drone_utils.step_coms( drones, 8001, { [ target, gsid, bitor( 0b01000000, 25 ), mid ] } );
% disp( append( "Success=", num2str( min( data{1} == [ gsid, target, 25, mid ] ) ) ) )