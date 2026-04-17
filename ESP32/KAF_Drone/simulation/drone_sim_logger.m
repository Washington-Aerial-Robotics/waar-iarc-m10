classdef drone_sim_logger < handle
    properties
        currfig, logtype, drones, times, logdata
    end
    methods
        function dologging( this, type, time, data )
            if strcmp( this.logtype, type )
                this.times = [ this.times, time ];
                this.logdata = [ this.logdata, data ];
            end
        end
        function this = drone_sim_logger( logtype, drones )
            this.currfig = figure;
            this.logtype = logtype;
            this.drones = drones;
            this.times = [];
            this.logdata = [];
        end
    end
end