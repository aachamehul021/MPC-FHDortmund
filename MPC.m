classdef MPC 
    % Simple MPC class
    % Most algorithms taken from Camacho, Bordons, model predictive control
    
    properties
        
        ref; % Reference
        smoothedRef; % Smoothed reference
        cost; % Cost function
     
        x; % State
        A; % System matrix
        B; % Control matrix
        C; % Measurement matrix
        % Matrizes of algo
        % See Camacho, Bordons, Model Predictive Control, second edition,
        % p. 29
        M; 
        N;
        Q;
        F;
        H;
        
        isSiSo; % Indicates if the system is a SiSo system 
        
        tvec; % Time vector
       

       
        ControlInput; % used to store ControlInput. Also used as "last control input" before it is beeing overwritten
        
        SysVer; % Version (see below)
        
        settings; % Settings from caller
        
        mode; % Mode (brute force or not)
        
        smoothingFactor;
   
        
    end
    
    methods
        
        function obj = MPC(settings,mode,n)
            
            obj.isSiSo = 1;
            obj.tvec =  settings.dt:settings.dt:settings.dt*settings.PredHorizonLength;
    
            obj.settings = settings;
            obj.mode = mode;
            obj.SysVer = settings.SysVer;
        
            obj.smoothingFactor = settings.smoothingFactor;
            
            switch obj.SysVer
                case 1
                    % SiSo system
                    obj.A = 1;
                    obj.B = settings.dt;
                    obj.C = 1;
                    obj.x = 0;
                case 2
                    % SiSo with two states
                    obj.A = [1 settings.dt;0 1];
                    obj.B = [settings.dt;1];
                    obj.C = [1 0];
                    obj.x = [0;0]; 
                case 3
                    % SiSo with two states / Measurement is a combination of both
                    obj.A = [1 settings.dt;0 1];
                    obj.B = [settings.dt;1];
                    obj.C = [1 0.1];
                    obj.x = [0;0]; 
       
                case 4
                    % SiMo
                    obj.isSiSo = 0;
                    obj.A = [1 settings.dt;0 1];
                    obj.B = [settings.dt;1];
                    obj.C = [1 0;0 1];
                    obj.x = [0;0];
            end
           
            predLength = settings.PredHorizonLength;
            L = length(obj.B);
            if ~exist('n','var')
                n = 1;
            else
                fprintf('Not tested for the case of u beeing not a scalar!\n');
            end

           
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Calc F and H
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            L0 = length(obj.C(:,1));
            if L0 >1
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % SiMo / MiMo case
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                fprintf('SiMo or MiMo model\n');
                M(1:L,1:L) = obj.A;
                M(1:L,L+1) = obj.B;
                M(L+1:L+n,L+1) = eye(n,1);
                N(1:L,1) = obj.B;
                N(L+1:L+n,1) = eye(n,1);
                Q = zeros(L,length(obj.C)+n);
                Q(1:L,1:L) = obj.C;
                F = zeros(predLength*(L),length(Q));
                for k = 1:predLength
                    n0 = (k-1)*(L)+1;
                    n1 = n0+(L)-1;
                    F(n0:n1,:) = Q*(M^k);
                end
                H = zeros(predLength,predLength);
                for rc = 1:predLength
                    rc1 = (rc-1)*(L)+1;
                    rc2 = rc1 + (L)-1;
                    
                    for cc = 1:rc
                        H(rc1:rc2,cc) = Q*M^(rc-cc)*N;
                    end
                end
            else
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % SiSo case
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                fprintf('SiSo case\n');
                M(1:L,1:L) = obj.A;
                M(1:L,L+1) = obj.B;
                M(L+1:L+n,L+1) = eye(n,1);
                N(1:L,1) = obj.B;
                N(L+1:L+n,1) = eye(n,1);
                Q = zeros(1,length(obj.C)+n);
                Q(1,1:L) = obj.C;
                F = zeros(predLength,length(Q));
                for k = 1:predLength
                    F(k,:) = Q*(M^k);
                end
                H = zeros(predLength,predLength);
                for rc = 1:predLength
                    for cc = 1:rc
                        H(rc,cc) = Q*M^(rc-cc)*N;
                    end
                end
            end
           
            obj.M = M;
            obj.N = N;
            obj.Q = Q;
            obj.F = F;
            obj.H = H;
            obj.ControlInput = 0;
                
        end
            
     
        function obj = update(obj,u)
            % Update the underlying model
            obj.x = obj.A*obj.x + obj.B*(u+obj.ControlInput);
        end
        
       

        function obj = SmoothReference(obj,vdes,v,ades,a)

             % Getting smoothed reference
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            dv = vdes-v;
            if obj.settings.smoothVer == 1
                smoothedV = v + dv*(1-exp(-obj.tvec*obj.smoothingFactor));
            else
                smoothedV = v + dv*(1:length(obj.tvec))/length(obj.tvec);
            end
 
            if length(obj.C(:,1)) > 1
                % Multiple output case: Intertwine
                da = ades-a;
                if obj.settings.smoothVer == 1
                    smoothedA = a + da*(1-exp(-obj.tvec*obj.smoothingFactor));
                else
                    smoothedA = a + da*(1:length(obj.tvec))/length(obj.tvec);
                end
                for k = 1:length(smoothedV)
                    obj.ref(2*k-1) = smoothedV(k);
                    obj.ref(2*k) = smoothedA(k);   
                end
             
            else
                obj.ref = smoothedV;
            end
            
          
        end
        
        
        function obj = costFunction(obj,vtmp,a,C1,C2,k)
            % Cost function for brute force
            obj.cost(k) = C1*sum(norm(vtmp-obj.ref))+ C2*sum(norm(a));
        end
        

        function obj = optInputBruteForce(obj)

           % Optimize brute force
           uvec = -10:0.01:10; % Test vector

           for k = 1:length(uvec) 
               % Loop over all elements of test vector
                tmp = obj.x;
                vtmp = zeros(1,obj.settings.PredHorizonLength);
                atmp = zeros(1,obj.settings.PredHorizonLength);
                vtmp(1) = tmp(1);
                atmp(1) = tmp(2);
                % Simulate system with constant control input
                for ik = 2:obj.settings.PredHorizonLength

                    tmp = obj.A*tmp + obj.B*uvec(k);
                    vtmp(ik) = tmp(1);
                    if length(tmp) > 1
                        atmp(ik) = tmp(2);
                    else
                        atmp(ik) = 0;
                    end
                end

                try 
                    % Note that for now, a is not taken into account at all
                    obj = obj.costFunction(vtmp,atmp,1,0,k);
                catch
                    lasterror
                end
           end
            % Find best fit
            p = find(obj.cost == min(obj.cost));
            obj.ControlInput = uvec(p(1)); 
           % fprintf('%4.2f ',uvec(p))
        
        end   
        
        
        function obj = OptInput(obj)
            % MPC algo
            xbar = [obj.x;obj.ControlInput]; % State vector extended with last control Input
            
            
            if obj.isSiSo == 0
                nom = obj.H'*(obj.ref'-obj.F*xbar);
                den = obj.H'*obj.H; %+ obj.settings.lambda*eye(size(obj.H));                    
            else
                   
                nom = obj.H'*(obj.ref'-obj.F*xbar);
                den = obj.H'*obj.H + obj.settings.lambda*eye(size(obj.H));
                   
            end
            
              delta_u = den\nom;
              obj.ControlInput = obj.ControlInput + delta_u(1);
                    
                
        end
            
        
        
        function obj = Update(obj, distortion)
            % Update function. To Do: Put smoothing here
            
            if obj.mode == 1
                obj = obj.OptInput();
            else
                obj = obj.optInputBruteForce();
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Apply constraint
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            if obj.ControlInput > obj.settings.MaxControl
                obj.ControlInput = obj.settings.MaxControl;
            end
            if obj.ControlInput < obj.settings.MinControl 
                obj.ControlInput = obj.settings.MinControl;
            end;
    
            
            % Apply system model
       
            obj = obj.update(distortion);
          
        end
       

    end
    
end

