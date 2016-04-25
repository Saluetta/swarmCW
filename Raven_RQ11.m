classdef Raven_RQ11
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    properties (SetAccess = private,GetAccess = private)
        pos;
        dir;
    end
    
    methods
        %Constructor
        function swarm = Raven_RQ11()
                swarm.pos = [0 0];
                swarm.dir = 0;
        end
        
        function move(bot,distance)
            
        end
        
        function setBotAng(bot,value)
            if(bot.adminKey ==0)
                bot.ang = value;
                bot.dir = [cos(bot.ang) sin(bot.ang)];
                bot.updateScanLines(0,1);
            else
                error('Incorrect key, cannot set bot angle during marking')
            end
        end
        
        
    end
    
end



