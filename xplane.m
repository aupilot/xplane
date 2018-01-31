classdef xplane < handle
    %XPLANE X-Plane control class
    %   Connection to X-Plane, write/read datarefs etc
    %   K.Makukhin @ auPilot Project
    %   license: MIT (do whatever you want) 
    
    % alternative from NASA: https://github.com/nasa/XPlaneConnect
    % list of commands: http://siminnovations.com/xplane/command/index.php
    % list of datarefs: http://siminnovations.com/xplane/dataref/index.php
    % quaternions: http://www.xpluginsdk.org/downloads/TestQuaternions.cpp
    
    % should we use this?: http://www.xsquawkbox.net/xpsdk/mediawiki/MovingThePlane#Positioning_the_User.27s_Aircraft_While_the_Physics_Engine_is_Running
    
    
    
    properties
        u
        
        N_ENGINES  = 3
        N_VECTORED = 2
        DATA_RATE  = 50     % times per second
        Gee = 9.81          % gravity at KSEA. your G might be different :)

        % egocentric reference frame
        Pitch =0;
        Roll  =0;
        Head  =0;
        Q = 0;
        P = 0;
        R = 0;
        Anorm = 0;
        Aaxil = 0;
        Aside = 0;
        
        % Global ref frame
%         X = 0;
%         Y = 0;
%         Z = 0;
%         Vx = 0;
%         Vy = 0;
%         Vz = 0;
%         Gear = zeros(1,3);
        Alt  = 0;   % m, above ground
%         Trav = 0;
        Lat  = 47.431;
        Lon  = -122.31;        
        Elv = 0;    % m, above sea level 
        
        % NED coordinates
        N 
        E
        D = 999
        Vn
        Ve
        Vd
%         An
%         Ae
%         Ad
        homeLat     % home position to calc NED from. Sets as first Lat/Lon arrived
        homeLon
        homeAlt
        
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
        
        % simSpeed -> simulator speedup:  0=paused, 1=normal, 2=double normal speed, 4, 8.
        function init(obj, simSpeed)
            %% init XPlane simulation
            
            % set sim_speed
            dataref = 'sim/time/sim_speed';
            obj.setDataref(simSpeed, dataref); 
            pause(0.1);            
            %% return to the runway
            obj.sendCmd('sim/operation/reset_to_runway');   
            pause(0.2);
            
            %% reset the view
%             obj.sendCmd('sim/view/circle');
            obj.sendCmd('sim/view/runway');
            pause(0.1);
                       
            %% let the throttle and surfaces to be overriden by our commands
            dataref = 'sim/operation/override/override_throttles';
            obj.setDataref(1, dataref);
            dataref = 'sim/operation/override/override_control_surfaces';
            obj.setDataref(1, dataref); 
            pause(0.1);
            
            %% just in case
            obj.setThrottle([0. 0. 0.])

        end
        
        function obj = dataReceivedCallBack(obj, uu, event)
            if event.Type == 'DatagramReceived' 
                try
                    datagram = fread(uu);
                    % parse the datagram
                    obj.parse(datagram);
    %                 disp(length(datagram))
                catch
                    warning('Broken packet received!\n');
                end

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
                
                Vx = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                Vy = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                Vz = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                
                obj.P = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                obj.Q = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                obj.R = typecast(datagram(ds:ds+SGL-1), 'single');
                
                % now update calculated NED position w.r.t. starting point
                if obj.D == 999  % first time - set the "home" point to calc NED from it
                    obj.homeLat = obj.Lat;
                    obj.homeLon = obj.Lon;
                    obj.homeAlt = obj.Alt;
                    obj.N = 0.0;
                    obj.E = 0.0;
                    obj.D = 0.0;
                else
                    [obj.N, obj.E, obj.D] = geodetic2ned(obj.Lat, obj.Lon, obj.homeAlt, obj.homeLat,obj.homeLon,obj.homeAlt, referenceEllipsoid('GRS 80'),'degrees');
                    obj.Ve = Vz;  % Vx;     % there is an error in XPlane docs? Their conversion does not seem to be right!
                    obj.Vn = Vx;  % -Vz;
                    obj.Vd = -Vy;
                end
                
            else
                if strcmp(char(datagram(1:4)'),'RREF') % we have multiple datarefs in one packet
                    len = length(datagram);
                    ds = 6;  % skip one byte. WTF is it? (decimal 44)
                    while ds < len
                        marker = typecast(datagram(ds:ds+U32-1), 'int32'); ds = ds+U32;
                        value  = typecast(datagram(ds:ds+SGL-1), 'single'); ds = ds+SGL;
                        switch marker
                            case 1
%                                 Ax = value;
                            case 2
%                                 Ay = value;
                            case 3
%                                 Az = value;
                            case 4
                                obj.Anorm = value * obj.Gee;                                
                            case 5
                                obj.Aaxil = value * obj.Gee;
                            case 6
                                obj.Aside = value * obj.Gee;
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
        % instead of the "right" way of controlling the vector (which only works for one motor).
        % To make this working - remove "vectors" ticks from engine spec in plainmaker!  
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
        
        function setApproach(obj, airport, throttle, vector)
            DBL = 8;
            U32 = 4;
            
            obj.setThrottle(throttle);
            obj.setVector(vector);

            packet2 = zeros(1,64+5, 'uint8');
            ppp = ['PREL' 0];
            
            packet2(1:length(ppp)) = ppp;
            ds=6; 
            packet2(ds:ds+U32-1) = typecast(int32(12),'uint8');  ds=ds+U32;   % loc_specify_lle          
            packet2(ds:ds+U32-1) = typecast(int32(0),'uint8');  ds=ds+U32;    % aircraft
            packet2(ds:ds+4-1) = airport;  ds=ds+8;                           % airport
            packet2(ds:ds+U32-1) = typecast(int32(0),'uint8');  ds=ds+U32;    % runway index
            packet2(ds:ds+U32-1) = typecast(int32(0),'uint8');  ds=ds+U32;    % runway direction    
            packet2(ds:ds+DBL-1) = typecast(double(0),'uint8');  ds=ds+DBL;
            packet2(ds:ds+DBL-1) = typecast(double(0),'uint8');  ds=ds+DBL;
            packet2(ds:ds+DBL-1) = typecast(double(0),'uint8');  ds=ds+DBL;
            packet2(ds:ds+DBL-1) = typecast(double(0),'uint8');  ds=ds+DBL;
            packet2(ds:ds+DBL-1) = typecast(double(15),'uint8');  
            fwrite(obj.u, packet2);
        end
        
        function setLocation(obj, lat, lon, ele, hea, spd)
            DBL = 8;
            U32 = 4;
            
            packet2 = zeros(1,64+5, 'uint8');
            ppp = ['PREL' 0];
            
            packet2(1:length(ppp)) = ppp;
            ds=6; 
            packet2(ds:ds+U32-1) = typecast(int32(6),'uint8');  ds=ds+U32;    % loc_specify_lle          
            packet2(ds:ds+U32-1) = typecast(int32(0),'uint8');  ds=ds+U32;    % aircraft
            packet2(ds:ds+8-1) = uint8(0);  ds=ds+8;                          % airport
            packet2(ds:ds+U32-1) = typecast(int32(0),'uint8');  ds=ds+U32;    % runway index
            packet2(ds:ds+U32-1) = typecast(int32(0),'uint8');  ds=ds+U32;    % runway direction    
            packet2(ds:ds+DBL-1) = typecast(double(lat),'uint8');  ds=ds+DBL;
            packet2(ds:ds+DBL-1) = typecast(double(lon),'uint8');  ds=ds+DBL;
            packet2(ds:ds+DBL-1) = typecast(double(ele),'uint8');  ds=ds+DBL;
            packet2(ds:ds+DBL-1) = typecast(double(hea),'uint8');  ds=ds+DBL;
            packet2(ds:ds+DBL-1) = typecast(double(spd),'uint8');  
            fwrite(obj.u, packet2);
        end
            
        function obj = request(obj)
            %REQUEST Request to subscribe to datastreams from x-plane
            %   if you need to stop processing - clear the class instance. 
            %   to stop XPlane sending streams - reboot XPLane!
            
            % This is the list of datarefs that XPlane might send back (not all of them of course). But we also need to parse them in callback!
            dataref{1} = 'sim/flightmodel/position/local_ax'; % this is in a global OpenGL ref frame and it does not include gravity.
            dataref{2} = 'sim/flightmodel/position/local_ay';
            dataref{3} = 'sim/flightmodel/position/local_az';
            dataref{4} = 'sim/flightmodel/forces/g_nrml';         
            dataref{5} = 'sim/flightmodel/forces/g_axil';  
            dataref{6} = 'sim/flightmodel/forces/g_side';
            
            useDatarefs = [4,5,6];  % we only subscribe for these
            
            %% subscribe 
            for i=useDatarefs
                packet2 = zeros(1,413);
                period  = int32(obj.DATA_RATE);
                marker  = int32(i);
                packet2(1:13+length(dataref{i})) = ['RREF' 0 '****' 'oooo' dataref{i}];  
                packet2(6:6+4-1)   = typecast(period,'uint8');
                packet2(10:10+4-1) = typecast(marker,'uint8');
                fwrite(obj.u, packet2);
                pause(0.02);
            end
            
            %% get position
            packet0 = ['RPOS',0,sprintf('%d',obj.DATA_RATE),0];  
            fwrite(obj.u, packet0);
            pause(0.02);
            
            %% initiate callbacks
            obj.u.ReadAsyncMode = 'continuous';
        end
        
    end
end

