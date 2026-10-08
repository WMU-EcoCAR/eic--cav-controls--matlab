function rosmsgOut = PointCloud2(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.header = bus_conv_fcns.ros2.busToMsg.std_msgs.Header(slBusIn.header,rosmsgOut.header(1));
    rosmsgOut.height = uint32(slBusIn.height);
    rosmsgOut.width = uint32(slBusIn.width);
    for iter=1:slBusIn.fields_SL_Info.CurrentLength
        rosmsgOut.fields(iter) = bus_conv_fcns.ros2.busToMsg.sensor_msgs.PointField(slBusIn.fields(iter),rosmsgOut.fields(1));
    end
    if slBusIn.fields_SL_Info.CurrentLength < numel(rosmsgOut.fields)
    rosmsgOut.fields(slBusIn.fields_SL_Info.CurrentLength+1:numel(rosmsgOut.fields)) = [];
    end
    rosmsgOut.is_bigendian = logical(slBusIn.is_bigendian);
    rosmsgOut.point_step = uint32(slBusIn.point_step);
    rosmsgOut.row_step = uint32(slBusIn.row_step);
    rosmsgOut.data = uint8(slBusIn.data(1:slBusIn.data_SL_Info.CurrentLength));
    rosmsgOut.is_dense = logical(slBusIn.is_dense);
end
