classdef flight_software_model
    methods( Static )
        function compile()
            mex -output drone_flight_software *.cpp ../core/*.cpp
            drone_flight_software( 'R' );
        end
        function init( args, sim, id )
            drone_flight_software( 'R' );
        end
        function outputdata = loopComs( time, inputdata )
            inputdata = uint8( inputdata )
            outputdata = drone_flight_software( 'C', time, inputdata )
        end
        function loopFlight( state, sim, id )
            drone_flight_software( 'F' );
        end
    end
end