function [P,M ,output] = rigidRegistration( source,target,varargin )
%  [P,M ,output] = pointToPointRegistration( sourcep_f,targetp_f,varargin )
% source_f     source image
% target_f     target image
% options:  'InitialParameters' can be a 6 element vector with initial
% guesses for angles and radiuses (in degrees)
%           'ParameterType' can be 'standard' or 'ultrasound'.
%
%
%   NOTE returns P in degres

P=[];
M=[];
%default values for options
initialParameters=[0 0 0 0 0 0];% tx ty tz a b c / a b h c d e
parameterType='standard';
writeToFile=false;
bruteForce =false;
outputFilename='none';
nOptions = size(varargin,2);
r = 150;
% for the standard
LB = [-100.0 -100.0 -100.0 -pi -pi -pi];   % Lower bound
UB = [100.0 100.0 100.0 pi pi pi];   % Upper bound
% for the ultrasoud
LB = [-pi -pi -100 -pi -pi -pi];   % Lower bound
UB = [pi  pi  100  pi  pi pi];   % Upper bound
padding_source=0;
padding_target=0;

%%
% Argument reading

if (nOptions > 0)
    i=1;
    while(i<=nOptions)
        
        if (strcmp(varargin{i},'InitialParameters'))
            initialParameters = varargin{i+1};
            disp(['Initial parameters set to ' num2str(initialParameters)])
        elseif (strcmp(varargin{i},'parameterBounds'))
            LB = varargin{i+1};
            UB = varargin{i+2};
            disp(['Parameter bounds set to ' num2str(LB) ', ' num2str(UB) ])
        elseif (strcmp(varargin{i},'padding_source'))
            padding_source= varargin{i+1};
            disp(['Padding value source ' num2str(padding_source)])
        elseif (strcmp(varargin{i},'padding_target'))
            padding_target = varargin{i+1};
            disp(['Padding value target' num2str(padding_target)])
        elseif  (strcmp(varargin{i},'writeToFile'))
            writeToFile=true;
            outputFilename = varargin{i+1};
            disp(['Output filename ' 'writeToFile' ]);
        elseif  (strcmp(varargin{i},'ParameterType'))
            parameterType = varargin{i+1};
            disp(['Parameter type ' parameterType ]);
        elseif  (strcmp(varargin{i},'bruteForce'))
            bruteForce =true;
            disp(['Parameter type ' parameterType ]);
        elseif  (strcmp(varargin{i},'r'))
            r = varargin{i+1};
            disp(['r ' parameterType ]);
        end
        i = i+1;
    end
end
%% minimise the 6 DOF cost function

if (strcmp(parameterType,'standard'))
    x0=initialParameters(:);
    
    
    % function: costFunction
    %Constrains: sx > 0, sy > 0, sz > 0 (positive scaling, reflections will be done through rotation as in rview)
    
    ObjectiveFunction = @(x)costFunctionStandard(x,target,source,padding_source,padding_target);
    nvars = 6;    % Number of variables: 3 rot, 3 trans, 3 scaling
    
    ConstraintFunction = @simple_constraint;
    %[x,fval] = ga(ObjectiveFunction,nvars,[],[],[],[],LB,UB,ConstraintFunction);
    %[x, fval] = fminunc(@costFunction,x0);
    %[x,fval] = patternsearch(ObjectiveFunction,x0,[],[],[],[],LB,UB, ConstraintFunction)
    %options=optimset('Algorithm','interior-point');
    options=optimset('TolFun',1e-10, 'TolCon', 1e-10,'TolX', 1e-10, ...
        'GradObj','off', 'Algorithm','interior-point','Display','iter-detailed');
    [x,fval, exitflag,output] = fmincon(ObjectiveFunction,x0,[],[],[],[],LB,UB, ConstraintFunction,options);
    
    
    tx=x(1);
    ty=x(2);
    tz=x(3);
    a=x(4);
    b=x(5);
    c=x(6);
    
    
    M = [cos(b)*cos(c)-sin(a)*sin(b)*sin(c) -cos(a)*sin(c) sin(a)*cos(b)*sin(c)+sin(b)*cos(c) -tx
        cos(b)*sin(c)+sin(a)*sin(b)*cos(c) cos(a)*cos(c) sin(b)*sin(c)-sin(a)*cos(b)*cos(c) -ty
        -cos(a)*sin(b) sin(a) cos(a)*cos(b) -tz
        0 0 0 1];
    
    %     M = [cos(b)*cos(c) -cos(a)*sin(c)-sin(a)*sin(b)*cos(c) sin(a)*sin(c)-cos(a)*sin(b)*cos(c) cos(c)*(cos(b)*tx-sin(b)*(cos(a)*tz+sin(a)*ty))-sin(c)*(cos(a)*ty-sin(a)*tz);
    %         cos(b)*sin(c) cos(a)*cos(c)-sin(a)*sin(b)*sin(c) -cos(a)*sin(b)*sin(c)-sin(a)*cos(c) sin(c)*(cos(b)*tx-sin(b)*(cos(a)*tz+sin(a)*ty))+cos(c)*(cos(a)*ty-sin(a)*tz);
    %         sin(b) sin(a)*cos(b) cos(a)*cos(b) cos(b)*(cos(a)*tz+sin(a)*ty)+sin(b)*tx;
    %         0 0 0 1];
    
    
    P = zeros(6,3);
    P(1,3)=tx;
    P(2,3)=ty;
    P(3,3)=tz;
    P(4,3)=a;
    P(5,3)=b;
    P(6,3)=c;
    
elseif (strcmp(parameterType,'ultrasound'))
    %%
    x0=initialParameters(:);
    
    ObjectiveFunction = @(x)costFunctionSpecial(x,target,source,r,padding_source,padding_target);
    if bruteForce
        
        dlt = [pi/18 pi/18 10 pi/18 pi/18 pi/18];
        
       [ x,output] = findBruteForceMin(ObjectiveFunction,LB,UB, dlt);
        
    else
        
        
        nvars = 3;    % Number of variables: 4 rot, 1 trans
        ConstraintFunction = @simple_constraint;
        %[x,fval] = ga(ObjectiveFunction,nvars,[],[],[],[],LB,UB,ConstraintFunction);
        %[x, fval] = fminunc(@costFunction,x0);
        %[x,fval] = patternsearch(ObjectiveFunction,x0,[],[],[],[],LB,UB, ConstraintFunction)
        
        %options=optimset('Algorithm','interior-point');
        %     options=optimset('TolFun',1e-06, 'TolCon', 1e-06,'TolX', 1e-10, ...
        %         'GradObj','off', 'Algorithm','interior-point','Display','iter',...
        %         'DiffMinChange',1e-02,'DiffMaxChange',45,'SubproblemAlgorithm','cg'); % interior-point
        
        options=optimset('TolFun',1e-10, 'TolCon', 1e-10,'TolX', 1e-10, ...
            'GradObj','off', 'Algorithm','interior-point','Display','iter'); %
        [x,fval, exitflag,output] = fmincon(ObjectiveFunction,x0,[],[],[],[],LB,UB, ConstraintFunction,options);
        
    end
    a=x(1);
    b=x(2);
    h=x(3);
    c=x(4);
    d=x(5);
    e=x(6);
    
    
    %     M = [-sin(a)*(sin(pi-b)*sin(pi-d)-sin(c)*cos(pi-b)*cos(pi-d))-cos(a)*cos(c)*cos(pi-d) sin(a)*cos(c)*cos(pi-b)+cos(a)*sin(c) -sin(a)*(-sin(c)*cos(pi-b)*sin(pi-d)-sin(pi-b)*cos(pi-d))-cos(a)*cos(c)*sin(pi-d) sin(a)*sin(pi-b)*(r+h)
    %         cos(a)*(sin(pi-b)*sin(pi-d)-sin(c)*cos(pi-b)*cos(pi-d))-sin(a)*cos(c)*cos(pi-d) sin(a)*sin(c)-cos(a)*cos(c)*cos(pi-b) cos(a)*(-sin(c)*cos(pi-b)*sin(pi-d)-sin(pi-b)*cos(pi-d))-sin(a)*cos(c)*sin(pi-d) -cos(a)*sin(pi-b)*(r+h)
    %         -cos(pi-b)*sin(pi-d)-sin(c)*sin(pi-b)*cos(pi-d),-cos(c)*sin(pi-b) cos(pi-b)*cos(pi-d)-sin(c)*sin(pi-b)*sin(pi-d) cos(pi-b)*(r+h)+r
    %         0 0 0 1];
    
    M = rreg_special_t1(r) * rreg_special_r1(a) * rreg_special_r2(b) * rreg_special_t2(r,h) * rreg_special_r3(c) * rreg_special_r4(d)* rreg_special_r4(e);
    
    
    P = zeros(6,3);
    P(1,3)=a;
    P(2,3)=b;
    P(3,3)=h;
    P(4,3)=c;
    P(5,3)=d;
    P(6,3)=e;
    
end

% write to file
if (writeToFile)
    fid = fopen(outputFilename, 'w');
    fprintf(fid, 'DOF: %d\n',trmode);
    for j=1:trmode
        fprintf(fid, '%d %d %f\n',P(j,1),P(j,2),P(j,3));
    end
    fclose(fid);
    
end

%% The following function will be minimised.
    function [val] = costFunctionSpecial( params, target, source, r, padding_source, padding_target )
        %params should be nine: 2 translation , 4 rotation .
        % params.*[180/pi 180/pi 1 180/pi 180/pi]'
        a=params(1);
        b=params(2);
        h=params(3);
        c=params(4);
        d=params(5);
        e=params(6);
        
        M = rreg_special_t1(r) * rreg_special_r1(a) * rreg_special_r2(b) * rreg_special_t2(r,h) * rreg_special_r3(c) * rreg_special_r4(d)* rreg_special_r4(e);
        source_oriented = setOrientation(source,M);
        valids = find(source_oriented.data>padding_source)';
        source_points = source_oriented.GetPosition(valids);
        values_target = target.GetValue(source_points,'linear');
        values_target(values_target<=padding_target)=NaN;
        values_source = source.data(valids);
        
%         source_tx = resampleImage(setOrientation(source,M),target,'interpolation','linear');
%         values_source =  source_tx.data(:);
%         values_target = target.data(:);
%         values_source(values_source<=padding_source)=NaN;
%         values_target(values_target<=padding_target)=NaN;
        
        val = (values_source - values_target).^2;
        val = nanmean(val);
        
        %         values_source =  source_tx.data;
        %         values_target = target.data;
        %         values_source(values_source<=padding_source)=NaN;
        %         values_target(values_target<=padding_target)=NaN;
        %
        %         val= nanmean(xcorr(values_source(:), values_target(:)));
        %
        
        
        if val~=val
            val=Inf;
        end
        %val = nansum(val);
        
    end

%% The following function will be minimised. Also returns the gradient
    function [val] = costFunctionStandard( params, target, source,padding_source, padding_target  )
        %params should be nine: 3translation, 3 rotation and 3 scaling (in that order).
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        
        % go!
        %         M = [cos(b)*cos(c) -cos(a)*sin(c)-sin(a)*sin(b)*cos(c) sin(a)*sin(c)-cos(a)*sin(b)*cos(c) cos(c)*(cos(b)*tx-sin(b)*(cos(a)*tz+sin(a)*ty))-sin(c)*(cos(a)*ty-sin(a)*tz);
        %             cos(b)*sin(c) cos(a)*cos(c)-sin(a)*sin(b)*sin(c) -cos(a)*sin(b)*sin(c)-sin(a)*cos(c) sin(c)*(cos(b)*tx-sin(b)*(cos(a)*tz+sin(a)*ty))+cos(c)*(cos(a)*ty-sin(a)*tz);
        %             sin(b) sin(a)*cos(b) cos(a)*cos(b) cos(b)*(cos(a)*tz+sin(a)*ty)+sin(b)*tx;
        %             0 0 0 1];
        M = [cos(b)*cos(c)-sin(a)*sin(b)*sin(c) -cos(a)*sin(c) sin(a)*cos(b)*sin(c)+sin(b)*cos(c) -tx
            cos(b)*sin(c)+sin(a)*sin(b)*cos(c) cos(a)*cos(c) sin(b)*sin(c)-sin(a)*cos(b)*cos(c) -ty
            -cos(a)*sin(b) sin(a) cos(a)*cos(b) -tz
            0 0 0 1];
        
        source_tx = resampleImage(setOrientation(source,M),target,'interpolation','linear');
        values_source =  source_tx.data(:)';
        
        %points = target.GetPosition(1:prod(target.size));
        %points_source = M * [points ; ones(1,size(points,2))];
        %values_source = source.GetValue(points_source(1:3,:),'linear');
        
        values_source(values_source<=padding_source)=NaN;
        values_target = target.data(:)';
        values_target(values_target<=padding_target)=NaN;
        
        val = (values_source - values_target).^2;
        
        val = nanmean(val);
        
        % calculate gradients
        
        %         g = zeros(1,9);
        %         g(1) = g_tx(params);
        %         g(2) = g_ty(params);
        %         g(3) = g_tz(params);
        %         g(4) = g_a(params);
        %         g(5) = g_b(params);
        %         g(6) = g_c(params);
        %         g(7) = g_sx(params);
        %         g(8) = g_sy(params);
        %         g(9) = g_sz(params);
        %
    end
% Gradients of the objective function
%% Gradient along tx
    function val = g_tx( params )
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        sx=params(7);
        sy=params(8);
        sz=params(9);
        val = 0;
        for i=1:num_of_points
            psx = sourcep(i,1);
            psy = sourcep(i,2);
            psz = sourcep(i,3);
            ptx = targetp(i,1);
            pty = targetp(i,2);
            ptz = targetp(i,3);
            val = val + 2*(tx-sin(b)*psz*sz+cos(b)*sin(c)*psy*sy+cos(b)*cos(c)*psx*sx-ptx);
        end
    end

%%
    function val = g_ty( params )
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        sx=params(7);
        sy=params(8);
        sz=params(9);
        val = 0;
        for i=1:num_of_points
            psx = sourcep(i,1);
            psy = sourcep(i,2);
            psz = sourcep(i,3);
            ptx = targetp(i,1);
            pty = targetp(i,2);
            ptz = targetp(i,3);
            val = val +2*(ty+sin(a)*cos(b)*psz*sz+(sin(a)*sin(b)*sin(c)+cos(a)*cos(c))*psy*sy+(sin(a)*sin(b)*cos(c)-cos(a)*sin(c))*psx*sx-pty);
        end
    end
%%
    function val = g_tz( params )
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        sx=params(7);
        sy=params(8);
        sz=params(9);
        val = 0;
        for i=1:num_of_points
            psx = sourcep(i,1);
            psy = sourcep(i,2);
            psz = sourcep(i,3);
            ptx = targetp(i,1);
            pty = targetp(i,2);
            ptz = targetp(i,3);
            val = val + 2*(tz+cos(a)*cos(b)*psz*sz+(cos(a)*sin(b)*sin(c)-sin(a)*cos(c))*psy*sy+(sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psx*sx-ptz);
        end
    end
%%
    function val = g_a( params )
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        sx=params(7);
        sy=params(8);
        sz=params(9);
        val = 0;
        for i=1:num_of_points
            psx = sourcep(i,1);
            psy = sourcep(i,2);
            psz = sourcep(i,3);
            ptx = targetp(i,1);
            pty = targetp(i,2);
            ptz = targetp(i,3);
            val = val + 2*(-sin(a)*cos(b)*psz*sz+(-sin(a)*sin(b)*sin(c)-cos(a)*cos(c))*psy*sy+(cos(a)*sin(c)-sin(a)*sin(b)*cos(c))*psx*sx)*...
                (tz+cos(a)*cos(b)*psz*sz+(cos(a)*sin(b)*sin(c)-sin(a)*cos(c))*psy*sy+(sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psx*sx-ptz)+2*...
                (cos(a)*cos(b)*psz*sz+(cos(a)*sin(b)*sin(c)-sin(a)*cos(c))*psy*sy+(sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psx*sx)*...
                (ty+sin(a)*cos(b)*psz*sz+(sin(a)*sin(b)*sin(c)+cos(a)*cos(c))*psy*sy+(sin(a)*sin(b)*cos(c)-cos(a)*sin(c))*psx*sx-pty);
        end
    end
%%
    function val = g_b( params )
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        sx=params(7);
        sy=params(8);
        sz=params(9);
        val = 0;
        for i=1:num_of_points
            psx = sourcep(i,1);
            psy = sourcep(i,2);
            psz = sourcep(i,3);
            ptx = targetp(i,1);
            pty = targetp(i,2);
            ptz = targetp(i,3);
            val = val + 2*(-cos(a)*sin(b)*psz*sz+cos(a)*cos(b)*sin(c)*psy*sy+cos(a)*cos(b)*cos(c)*psx*sx)*...
                (tz+cos(a)*cos(b)*psz*sz+(cos(a)*sin(b)*sin(c)-sin(a)*cos(c))*psy*sy+(sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psx*sx-ptz)+2*...
                (-sin(a)*sin(b)*psz*sz+sin(a)*cos(b)*sin(c)*psy*sy+sin(a)*cos(b)*cos(c)*psx*sx)*...
                (ty+sin(a)*cos(b)*psz*sz+(sin(a)*sin(b)*sin(c)+cos(a)*cos(c))*psy*sy+(sin(a)*sin(b)*cos(c)-cos(a)*sin(c))*psx*sx-pty)+2*...
                (-cos(b)*psz*sz-sin(b)*sin(c)*psy*sy-sin(b)*cos(c)*psx*sx)*(tx-sin(b)*psz*sz+cos(b)*sin(c)*psy*sy+cos(b)*cos(c)*psx*sx-ptx);
        end
    end
%%
    function val = g_c( params )
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        sx=params(7);
        sy=params(8);
        sz=params(9);
        val = 0;
        for i=1:num_of_points
            psx = sourcep(i,1);
            psy = sourcep(i,2);
            psz = sourcep(i,3);
            ptx = targetp(i,1);
            pty = targetp(i,2);
            ptz = targetp(i,3);
            val = val + 2*((sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psy*sy+(sin(a)*cos(c)-cos(a)*sin(b)*sin(c))*psx*sx)*...
                (tz+cos(a)*cos(b)*psz*sz+(cos(a)*sin(b)*sin(c)-sin(a)*cos(c))*psy*sy+(sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psx*sx-ptz)+2*...
                ((sin(a)*sin(b)*cos(c)-cos(a)*sin(c))*psy*sy+(-sin(a)*sin(b)*sin(c)-cos(a)*cos(c))*psx*sx)*...
                (ty+sin(a)*cos(b)*psz*sz+(sin(a)*sin(b)*sin(c)+cos(a)*cos(c))*psy*sy+(sin(a)*sin(b)*cos(c)-cos(a)*sin(c))*psx*sx-pty)+2*(cos(b)*cos(c)*psy*sy-cos(b)*sin(c)*psx*sx)*...
                (tx-sin(b)*psz*sz+cos(b)*sin(c)*psy*sy+cos(b)*cos(c)*psx*sx-ptx);
        end
    end
%%
    function val = g_sx( params )
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        sx=params(7);
        sy=params(8);
        sz=params(9);
        val = 0;
        for i=1:num_of_points
            psx = sourcep(i,1);
            psy = sourcep(i,2);
            psz = sourcep(i,3);
            ptx = targetp(i,1);
            pty = targetp(i,2);
            ptz = targetp(i,3);
            val = val + 2*(sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psx*(tz+cos(a)*cos(b)*psz*sz+(cos(a)*sin(b)*sin(c)-sin(a)*cos(c))*psy*sy+(sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psx*sx-ptz)+2*...
                (sin(a)*sin(b)*cos(c)-cos(a)*sin(c))*psx*(ty+sin(a)*cos(b)*psz*sz+(sin(a)*sin(b)*sin(c)+cos(a)*cos(c))*psy*sy+(sin(a)*sin(b)*cos(c)-cos(a)*sin(c))*psx*sx-pty)+2*cos(b)*cos(c)*...
                psx*(tx-sin(b)*psz*sz+cos(b)*sin(c)*psy*sy+cos(b)*cos(c)*psx*sx-ptx);
        end
    end
%%
    function val = g_sy( params )
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        sx=params(7);
        sy=params(8);
        sz=params(9);
        val = 0;
        for i=1:num_of_points
            psx = sourcep(i,1);
            psy = sourcep(i,2);
            psz = sourcep(i,3);
            ptx = targetp(i,1);
            pty = targetp(i,2);
            ptz = targetp(i,3);
            val = val + 2*(cos(a)*sin(b)*sin(c)-sin(a)*cos(c))*psy*(tz+cos(a)*cos(b)*psz*sz+(cos(a)*sin(b)*sin(c)-sin(a)*cos(c))*psy*sy+(sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psx*sx-ptz)+2*...
                (sin(a)*sin(b)*sin(c)+cos(a)*cos(c))*psy*(ty+sin(a)*cos(b)*psz*sz+(sin(a)*sin(b)*sin(c)+cos(a)*cos(c))*psy*sy+(sin(a)*sin(b)*cos(c)-cos(a)*sin(c))*psx*sx-pty)+2*cos(b)*sin(c)*...
                psy*(tx-sin(b)*psz*sz+cos(b)*sin(c)*psy*sy+cos(b)*cos(c)*psx*sx-ptx);
        end
    end
%%
    function val = g_sz( params )
        tx=params(1);
        ty=params(2);
        tz=params(3);
        a=params(4);
        b=params(5);
        c=params(6);
        sx=params(7);
        sy=params(8);
        sz=params(9);
        val = 0;
        for ii=1:num_of_points
            psx = sourcep(ii,1);
            psy = sourcep(ii,2);
            psz = sourcep(ii,3);
            ptx = targetp(ii,1);
            pty = targetp(ii,2);
            ptz = targetp(ii,3);
            val = val + 2*cos(a)*cos(b)*psz*(tz+cos(a)*cos(b)*psz*sz+(cos(a)*sin(b)*sin(c)-sin(a)*cos(c))*psy*sy+(sin(a)*sin(c)+cos(a)*sin(b)*cos(c))*psx*sx-ptz)+2*sin(a)*cos(b)*psz*...
                (ty+sin(a)*cos(b)*psz*sz+(sin(a)*sin(b)*sin(c)+cos(a)*cos(c))*psy*sy+(sin(a)*sin(b)*cos(c)-cos(a)*sin(c))*psx*sx-pty)-2*sin(b)*psz*...
                (tx-sin(b)*psz*sz+cos(b)*sin(c)*psy*sy+cos(b)*cos(c)*psx*sx-ptx);
        end
    end
%%
%constrain function
    function [c, ceq] = simple_constraint(params)
        c = [];
        ceq = [];
    end


    function [x,output] = findBruteForceMin(ObjectiveFunction,LB,UB,dlt)
        
        
        
        [a,b,h,c,d,e]=ndgrid(LB(1):dlt(1):UB(1),LB(2):dlt(2):UB(2),LB(3):dlt(3):UB(3),LB(4):dlt(4):UB(4),LB(5):dlt(5):UB(5),LB(6):dlt(6):UB(6));
        
        params = [a(:) b(:) h(:) c(:) d(:) e(:)];
        
        vals = zeros(size(params,1),1);
        distcomp.feature( 'LocalUseMpiexec', false );
%         parobj = parpool(2);
%         disp('Parpool started')
%         parfor id=1:size(params,1)
        for id=1:size(params,1)
            vals(id)=ObjectiveFunction(params(id,:));
            
            if ~mod(id,ceil(size(params,1)/100))
                disp([ num2str(id) ' of ' num2str(size(params,1))])
            end
        end
%        delete(parobj);
        
        [minval,idx]=min(vals);
        output.minval = minval;
        output.mintype = 'brute force';
        x = params(idx,:);
        
    end

end



