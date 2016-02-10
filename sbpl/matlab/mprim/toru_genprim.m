% /*
%  * Copyright (c) 2008, Maxim Likhachev
%  * All rights reserved.
%  * 
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions are met:
%  * 
%  *     * Redistributions of source code must retain the above copyright
%  *       notice, this list of conditions and the following disclaimer.
%  *     * Redistributions in binary form must reproduce the above copyright
%  *       notice, this list of conditions and the following disclaimer in the
%  *       documentation and/or other materials provided with the distribution.
%  *     * Neither the name of the University of Pennsylvania nor the names of its
%  *       contributors may be used to endorse or promote products derived from
%  *       this software without specific prior written permission.
%  * 
%  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  * POSSIBILITY OF SUCH DAMAGE.
%  */
function[] = toru_genprim(outfilename)

%
%generates motion primitives and saves them into file
%
%written by Maxim Likhachev
%---------------------------------------------------
%

%defines

resolution = 0.015;              % this parameter should match the current map resolution
max_simulation_time = 0.75;     % maximal time to simulate trajectory forward. The lowest the more precise the planning will be
numberofangles = 16;            %preferably a power of 2, definitely multiple of 8
eps = 0.1;                      % plus/minus percentege range for search length (search length = max_sim_time * linear_speed)

delta_theta = 2*pi/numberofangles / max_simulation_time;

% speeds in m/s or rad/s
FAST_LINEAR_SPEED = 0.5;
SLOW_LINEAR_SPEED = 0.1;
LINEAR_SPEED_WITH_ROTATION = 0.35;
FAST_ROTATION = 2 * delta_theta;
SLOW_ROTATION = 1 * delta_theta;
ROTATION_IN_PLACE = 2 * delta_theta;
INERTIAL_OFFSET = 0.1 % constant used to take into account the inertia of the robot when it starts from rest.
                      % In this case in fact, the robot will move straight for a little bit before starting to turn.

%multipliers (multiplier is used as costmult*cost)
COSTMULT_STRAIGHT = 1;
COSTMULT_TURN_MOVING_FORWARD = 1;
COSTMULT_TURN_IN_PLACE = 1;


% PRIMTIVES parameters [v_x, v_theta, costmult, inertial_offset]

primitives_params = ...
    [FAST_LINEAR_SPEED, 0, COSTMULT_STRAIGHT, 0;...
    SLOW_LINEAR_SPEED, 0, COSTMULT_STRAIGHT, 0;...
    -FAST_LINEAR_SPEED, 0, COSTMULT_STRAIGHT, 0;...
    -SLOW_LINEAR_SPEED, 0, COSTMULT_STRAIGHT, 0;...
     LINEAR_SPEED_WITH_ROTATION, FAST_ROTATION, COSTMULT_TURN_MOVING_FORWARD, INERTIAL_OFFSET;...
    LINEAR_SPEED_WITH_ROTATION, -FAST_ROTATION, COSTMULT_TURN_MOVING_FORWARD, INERTIAL_OFFSET;...
    -LINEAR_SPEED_WITH_ROTATION, FAST_ROTATION, COSTMULT_TURN_MOVING_FORWARD, INERTIAL_OFFSET;...
    -LINEAR_SPEED_WITH_ROTATION, -FAST_ROTATION, COSTMULT_TURN_MOVING_FORWARD, INERTIAL_OFFSET;...
    LINEAR_SPEED_WITH_ROTATION, SLOW_ROTATION, COSTMULT_TURN_MOVING_FORWARD, INERTIAL_OFFSET;...
    LINEAR_SPEED_WITH_ROTATION, -SLOW_ROTATION, COSTMULT_TURN_MOVING_FORWARD, INERTIAL_OFFSET;...
    -LINEAR_SPEED_WITH_ROTATION, SLOW_ROTATION, COSTMULT_TURN_MOVING_FORWARD, INERTIAL_OFFSET;...
    -LINEAR_SPEED_WITH_ROTATION, -SLOW_ROTATION, COSTMULT_TURN_MOVING_FORWARD, INERTIAL_OFFSET;...
    0, ROTATION_IN_PLACE, COSTMULT_TURN_IN_PLACE, 0;...
    0, -ROTATION_IN_PLACE, COSTMULT_TURN_IN_PLACE, 0;...
    ];

numofsamples = 10;              % number of points in the lattice. (it shouldn't be modified)
numberofprimsperangle = size(primitives_params, 1);

fout = fopen(outfilename, 'w');


%write the header
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);


%iterate over angles
for angleind = 1:numberofangles
    figure(1);
    hold off;

    %current angle
    current_angle = (angleind-1)*2*pi/numberofangles;
    % currentangle_36000int = round((angleind-1)*36000/numberofangles);
        
    text(0, 0, int2str(angleind));
    
    for prim_idx = 1:numberofprimsperangle
        % Iterate through primitive params to get endpoints
        forward_speed = primitives_params(prim_idx,1);
        angular_speed = primitives_params(prim_idx,2);
        offset = primitives_params(prim_idx,4);

        if forward_speed == 0
            x = 0;
            y = 0;
            theta = angular_speed + current_angle;
        else
            [x, y, theta] = gen_unicycle_endpoint(current_angle, forward_speed, angular_speed, max_simulation_time, resolution, eps, offset);
        end
        fprintf(fout, 'primID: %d\n', prim_idx-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);
        angle = 0;

        %now figure out what action will be        
        baseendpose_c = [round(x/resolution), round(y/resolution), round(theta / (2*pi) * numberofangles)-(angleind-1)];
        additionalactioncostmult = primitives_params(prim_idx,3);
        endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));
        endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        %round(theta / 2*pi * numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];
        
        fprintf(1, 'rotation angle=%f\n', angle*180/pi);
        
        %generate intermediate poses (remember they are w.r.t 0,0 (and not
        %centers of the cells)
        intermcells_m = zeros(numofsamples,3);
        if 1 == 1
            startpt = [0 0 current_angle];
            endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
                rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles];
            intermcells_m = zeros(numofsamples,3);
            if ((endx_c == 0 && endy_c == 0) || baseendpose_c(3) == 0) %turn in place or move forward            
                for iind = 1:numofsamples
                    intermcells_m(iind,:) = [startpt(1) + (endpt(1) - startpt(1))*(iind-1)/(numofsamples-1) ...
                                            startpt(2) + (endpt(2) - startpt(2))*(iind-1)/(numofsamples-1) ...
                                            0];
                    rotation_angle = (baseendpose_c(3) ) * (2*pi/numberofangles);
                    intermcells_m(iind,3) = rem(startpt(3) + (rotation_angle)*(iind-1)/(numofsamples-1), 2*pi);
                end;            
            else %unicycle-based move forward or backward
                R = [cos(startpt(3)) sin(endpt(3)) - sin(startpt(3));
                    sin(startpt(3)) -(cos(endpt(3)) - cos(startpt(3)))];
                S = pinv(R)*[endpt(1) - startpt(1); endpt(2) - startpt(2)];
                l = S(1); 
                tvoverrv = S(2);
                rv = (baseendpose_c(3)*2*pi/numberofangles + l/tvoverrv);
                tv = tvoverrv*rv;
                         
                if ((l < 0 & tv > 0) | (l > 0 & tv < 0))
                    fprintf(1, 'WARNING: l = %d < 0 -> bad action start/end points\n', l);
                    l = 0;
                end;
                %compute rv
                %rv = baseendpose_c(3)*2*pi/numberofangles;
                %compute tv
                %tvx = (endpt(1) - startpt(1))*rv/(sin(endpt(3)) - sin(startpt(3)))
                %tvy = -(endpt(2) - startpt(2))*rv/(cos(endpt(3)) - cos(startpt(3)))
                %tv = (tvx + tvy)/2.0;              
                %generate samples
                for iind = 1:numofsamples                                        
                    dt = (iind-1)/(numofsamples-1);
                                        
                    %dtheta = rv*dt + startpt(3);
                    %intermcells_m(iind,:) = [startpt(1) + tv/rv*(sin(dtheta) - sin(startpt(3))) ...
                    %                        startpt(2) - tv/rv*(cos(dtheta) - cos(startpt(3))) ...
                    %                        dtheta];
                    
                    if(abs(dt*tv) < abs(l))
                        intermcells_m(iind,:) = [startpt(1) + dt*tv*cos(startpt(3)) ...
                                                 startpt(2) + dt*tv*sin(startpt(3)) ...
                                                 startpt(3)];
                    else
                        dtheta = rv*(dt - l/tv) + startpt(3);
                        intermcells_m(iind,:) = [startpt(1) + l*cos(startpt(3)) + tvoverrv*(sin(dtheta) - sin(startpt(3))) ...
                                                 startpt(2) + l*sin(startpt(3)) - tvoverrv*(cos(dtheta) - cos(startpt(3))) ...
                                                 dtheta];
                    end;
                end; 
                %correct
                errorxy = [endpt(1) - intermcells_m(numofsamples,1) ... 
                           endpt(2) - intermcells_m(numofsamples,2)];
                fprintf(1, 'l=%f errx=%f erry=%f\n', l, errorxy(1), errorxy(2));
                interpfactor = [0:1/(numofsamples-1):1];
                intermcells_m(:,1) = intermcells_m(:,1) + errorxy(1)*interpfactor';
                intermcells_m(:,2) = intermcells_m(:,2) + errorxy(2)*interpfactor';
            end;                                        
        end;
    
        %write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3));
        end;
        
        plot(intermcells_m(:,1), intermcells_m(:,2));
        axis([-0.3 0.3 -0.3 0.3]);
        text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
        hold on;
        
    end;

    grid;
    pause;
end;
        
fclose('all');
