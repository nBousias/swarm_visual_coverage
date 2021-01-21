function [N,X,Y,Z,TH,HI,Xs,Ys,Zs,THs,HIs,ycs,xcs,as,bs,C,W,uX,uY,uZ,uz,uTH,uHI,uD,xcd,ycd,ad,bd,ycds,xcds,ads,bds,D,Ds]=agent_failure(smax,N,X,Y,Z,TH,HI,Xs,Ys,Zs,THs,HIs,ycs,xcs,as,bs,C,W,uX,uY,uZ,uz,uTH,uHI,uD,xcd,ycd,ad,bd,ycds,xcds,ads,bds,D,Ds)
    
        N=N-2;
        uz=uz(1:N);
        X = X(1:N);
        Y = Y(1:N);
        Z = Z(1:N);
        TH = TH(1:N);
        HI= HI(1:N);
        D= D(1:N);
           
        Xs = zeros(smax, N);
        Ys = zeros(smax, N);
        Zs = zeros(smax, N);
        THs = zeros(smax, N);
        HIs = zeros(smax, N);
        Ds = zeros(smax, N);
        ycs = zeros(smax, N);
        xcs = zeros(smax, N);
        as = zeros(smax, N);
        bs = zeros(smax, N);
        C = cell([1 N]);
        W = cell([1 N]);
        uX = zeros(1,N);
        uY = zeros(1,N);
        uZ = zeros(1,N);
        uTH = zeros(1,N);
        uHI = zeros(1,N);
        
        uD = zeros(1,N);
        xcds= zeros(smax, N);
        ycds= zeros(smax, N);
        ads= zeros(smax, N);
        bds= zeros(smax, N);
        
        xcd= zeros(smax, N);
        ycd= zeros(smax, N);
        ad= zeros(smax, N);
        bd= zeros(smax, N);


end

