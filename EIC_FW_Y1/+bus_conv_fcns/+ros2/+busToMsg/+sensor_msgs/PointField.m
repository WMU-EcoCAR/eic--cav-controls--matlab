function rosmsgOut = PointField(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.name = char(slBusIn.name);
    if slBusIn.name_SL_Info.CurrentLength < numel(slBusIn.name)
    rosmsgOut.name(slBusIn.name_SL_Info.CurrentLength+1:numel(slBusIn.name)) = [];
    end
    rosmsgOut.offset = uint32(slBusIn.offset);
    rosmsgOut.datatype = uint8(slBusIn.datatype);
    rosmsgOut.count = uint32(slBusIn.count);
end
