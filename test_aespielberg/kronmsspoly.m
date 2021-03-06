function K = kronmsspoly(A,B)
    %simplified version of MATLAB's kron which will work for msspoly's

    ASize = size(A);
    BSize = size(B);
    
    An = ASize(1);
    Bn = BSize(1);
    Am = ASize(2);
    Bm = BSize(2);
    
    Kn = An * Bn;
    Km = Am * Bm;
    
    K = zeros(Kn, Km);
    
    if isa(A, 'msspoly') || isa(B, 'msspoly')
        K = msspoly(K);
        
    elseif isa(A, 'TrigPoly') || isa(B, 'TrigPoly')
        K = msspoly(K);
        
    elseif isa(A, 'sym') || isa(B, 'sym')
        K = sym(K);
        
    end

    for iA = 1:1:An
        for iB = 1:1:Bn
            for jA = 1:1:Am
                for jB = 1:1:Bm

                    K((iA - 1) * Bn + iB, (jA - 1) * Bm + jB) = A(iA, jA) * B(iB, jB);
                    
                end
            end
        end
    end
    
end