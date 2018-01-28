classdef xplane < handle
    %XPLANE X-Plane control class
    %   Connection to X-Plane, write/read datarefs etc
    
    % alternative from NASA: https://github.com/nasa/XPlaneConnect
    % list of commands: http://siminnovations.com/xplane/command/index.php
    % list of datarefs: http://siminnovations.com/xplane/dataref/index.php
    
    properties
        u
        
        N_ENGINES  = 3
        N_VECTORED = 2
        DATA_RATE  = 20  % times per second
        SIM_SPEEDX = 1

        Pitch =0;
        Roll  =0;
        Head  =0;
        Vx = 0;
        Vy = 0;
        Vz = 0;
        Q = 0;
        P = 0;
        R = 0;
        Ax = 0;
        Ay = 0;
        Az = 0;
%         X = 0;
%         Y = 0;
%         Z = 0;
%         Gear = zeros(1,3);
        Alt  = 0;   % m, above ground
        Trav = 0;
        Lat  = 47.431;
        Lon  = -122.31;        
        Elv = 0;    % m, above sea level   
    end
    
    methods
        function obj = xplane()
            %XPLANE Construct an instance of this class
            %   Connect to XPlane

            uAll = instrfindall;
            delete(uAll);

            % we don't need to set any network settings in X_Plane!
            obj.u = udp('192.168.1.111', 'LocalPort', 49005, 'RemoteHost', '192.168.1.111', 'RemotePort', 49000);  % send to the sim Local port 49000 from any and receive to the any port
            obj.u.ReadAsyncMode = 'manual';     % we'll run it later
            obj.u.DatagramTerminateMode='On';
            obj.u.DatagramReceivedFcn = @obj.dataReceivedCallBack;
            fopen(obj.u);
        end
        
        function init(obj)
            %% init XPlane simulation
            
            % set sim_speed  0=paused, 1=normal, 2=double normal speed, 4, 8.
%             dataref = 'sim/time/sim_speed';
%             obj.setDataref(obj.SIM_SPEEDX, dataref); 
            
            %% return to the runway
            obj.sendCmd('sim/operation/reset_to_runway');   
            pause(0.25);
                       
            %% reset the view
%             obj.sendCmd('sim/view/circle');
            obj.sendCmd('sim/view/runway');
            pause(0.2);
                       
            %% let the throttle and surfaces to be overriden by our commands
            dataref = 'sim/operation/override/override_throttles';
            obj.setDataref(1, dataref);
            dataref = 'sim/operation/override/override_control_surfaces';
            obj.setDataref(1, dataref); 

        end
        
        function obj = dataReceivedCallBack(obj, uu, event)
            if event.Type == 'DatagramReceived' 
                datagram = fread(uu);
                % parse the datagram
                obj.parse(datagram);
%                 disp(length(datagram))
            else
                disp(event.Type);
            end
        end
        
        function obj = parse(obj, datagram)
            datagram = uint8(datagram);
            SGL = 4;
            DBL = 8;
            U32 = 4;
            if strcmp(char(datagram(1:4)'),'RPOS')
                ds = 6; 
                obj.Lat = typecast(datagram(ds:ds+DBL-1), 'double'); ds = ds+DBL;
                obj.Lon = typecast(datagram(ds:ds+DBL-1), 'double'); ds = ds+DBL;
                obj.Elv = typecast(datagram(ds:ds+DBL-1), 'double'); ds = ds+DBL;
                obj.Alt = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                
                obj.Pitch = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                obj.Head = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                obj.Roll = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                
                obj.Vx = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                obj.Vy = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                obj.Vz = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                
                obj.P = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                obj.Q = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                obj.R = typecast(datagram(ds:ds+SGL-1), 'single');
            else
                if strcmp(char(datagram(1:4)'),'RREF') % we have multiple datarefs in one packet
                    len = length(datagram);
                    ds = 6;  % skip one byte. WTF is it? (decimal 44)
                    while ds < len
                        marker = typecast(datagram(ds:ds+U32-1), 'int32'); ds = ds+U32;
                        value  = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                        switch marker
                            case 1
                                obj.Ax = value;
                            case 2
                                obj.Ay = value;
                            case 3
                                obj.Az = value;
                            case 0      % should not get here
                                break;
                            otherwise
                                warning('Wrong dataref marker %d %f', marker, value);
                        end
                    end
                else
                    warning('Wrong packet ID');  
                end
            end
            
        end

        function setThrottle(obj, thro)
            for i=1:obj.N_ENGINES
                dataref = sprintf('sim/flightmodel/engine/ENGN_thro_use[%d]', i-1);
                obj.setDataref(thro(i), dataref);  
            end
        end
        
        % It is tricky if you have more than one engines. We are changing the engin geometry 
        % instead of the "right" way of contolling the vector (which only works for one motor).
        % To make this working - remove "vectors" ticks from engin spec in plainmaker!  
        % 0  -> forward
        % 90 -> up
        function setVector(obj, vect)
            for i=1:obj.N_VECTORED
%                 dataref = sprintf('sim/flightmodel2/engines/nacelle_vertical_angle_deg[%d]', i-1);
%                 obj.setDataref(vect(i), dataref);  
%                 dataref = sprintf('sim/flightmodel2/engines/rotor_vertical_angle_deg[%d]', i-1);              
%                 obj.setDataref(vect(i), dataref);  
                dataref = sprintf('sim/aircraft/prop/acf_vertcant[%d]', i-1);
                obj.setDataref(vect(i), dataref); 
            end
        end
        
        function setDataref(obj, value, dataref)
            packet2 = zeros(1,509);
            sendValue  = single(value);
            packet2(1:9+length(dataref)) = ['DREF' 0 '****' dataref];  
            packet2(6:6+4-1)   = typecast(sendValue,'uint8');
            fwrite(obj.u, packet2);
        end
        
        function sendCmd(obj, cmd)
            packet2 = zeros(1,509);
            packet2(1:5+length(cmd)) = ['CMND' 0 cmd];  
            fwrite(obj.u, packet2);
        end   
        
        function obj = request(obj)
            %REQUEST Request to subscribe to datastreams from x-plane
            %   if you need to stop processing - clear the class instance. 
            %   to stop XPlane sending streams - reboot XPLane!
            
            %% get accelerations
            packet2 = zeros(1,413);
            dataref = 'sim/flightmodel/position/local_ax';
            period  = int32(obj.DATA_RATE);
            marker  = int32(1);
            packet2(1:13+length(dataref)) = ['RREF' 0 '****' 'oooo' dataref];  
            packet2(6:6+4-1)   = typecast(period,'uint8');
            packet2(10:10+4-1) = typecast(marker,'uint8');
            fwrite(obj.u, packet2);
            pause(0.02);
            packet2 = zeros(1,413);
            dataref = 'sim/flightmodel/position/local_ay';
            period  = int32(obj.DATA_RATE);
            marker  = int32(2);
            packet2(1:13+length(dataref)) = ['RREF' 0 '****' '****' dataref];  
            packet2(6:6+4-1)   = typecast(period,'uint8');
            packet2(10:10+4-1) = typecast(marker,'uint8');
            fwrite(obj.u, packet2);
            pause(0.02);
            packet2 = zeros(1,413);
            dataref = 'sim/flightmodel/position/local_az';
            period  = int32(obj.DATA_RATE);
            marker  = int32(3);
            packet2(1:13+length(dataref)) = ['RREF' 0 '****' '****' dataref];  
            packet2(6:6+4-1)   = typecast(period,'uint8');
            packet2(10:10+4-1) = typecast(marker,'uint8');
            fwrite(obj.u, packet2);
            pause(0.02);

            %% get position
            packet0 = ['RPOS',0,sprintf('%d',obj.DATA_RATE),0];  
            fwrite(obj.u, packet0);
            pause(0.02);
            
            obj.u.ReadAsyncMode = 'continuous';
        end
        
    end
end

