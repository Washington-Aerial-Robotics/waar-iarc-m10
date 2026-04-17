clear; clc; close all;
coords = [ 0, 1, 1, 1, 0, -1, -1, -1, 0,  1,  1,  1, 0, -1, -1, -1, 0, 0, 0; 
           0, 1, 1, 1, 0,  1,  1,  1, 0, -1, -1, -1, 0, -1, -1, -1, 0, 0, 0; 
           0, 0, 1, 0, 0,  0,  1,  0, 0,  0,  1,  0, 0,  0,  1,  0, 0, 1, 0 ];
figure;
view( 3 );
plt = plot3( 0, 0, 0, ".-", "Color", "black" );
xlim( [ -1.3, 1.3 ] );
ylim( [ -1.3, 1.3 ] );
zlim( [ -1.3, 1.3 ] );
drone = serialport( "COM9", 115200 );
thisID = 'G';
targetID = 'U';
messageID = 1;
pause( 2 );
if drone.NumBytesAvailable > 0
    disp( char( drone.read( drone.NumBytesAvailable, "uint8" ) ) )
end
while plt.isvalid
    messageID = mod( messageID + 1, 256 );
    drone.write( uint8( [ targetID, thisID, bitor( 0b01000000, 28 ), messageID ] ), "uint8" );
    data = drone.read( 10, "single" );
    R = [ data( 2 ), data( 3 ), data( 4 ); data( 5 ), data( 6 ) data( 7 ); data( 8 ), data( 9 ), data( 10 ) ];
    drone.write( uint8( [ targetID, thisID, bitor( 0b01000000, 6 ), messageID ] ), "uint8" );
    data = drone.read( 5, "single" );
    coords( 3, [ 3, 7, 11, 15 ] ) = data( 2:5 );
    tcoords = R * coords;
    try
        plt.XData = tcoords( 1, : );
        plt.YData = tcoords( 2, : );
        plt.ZData = tcoords( 3, : );
    catch
    end
end
drone.delete();
disp( "FINISHED" )