function delta = normDelta(inputDelta)
    function a = normAng(ang)
        a = mod(ang, 2*pi);
        a = mod((a+2*pi),2*pi);
    end
delta = normAng(inputDelta);
if delta > pi
    delta = delta - 2*pi;
end

end

