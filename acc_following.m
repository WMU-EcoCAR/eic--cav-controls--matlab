function following = acc_following(out, P)
%ACC_FOLLOWING True at each logged step where ACC is following the lead.
%   following = ACC_FOLLOWING(out, P) uses the PID model's logged mode when
%   it exists. MPC has no explicit mode, so for it the lead counts as
%   followed when it is in radar range and the spacing policy asks for a
%   speed below the set speed: the same rule the PID arbitration uses.

if any(strcmp(out.who, 'mode'))
    following = out.mode.Data(:) == 2;
    return
end
gap = out.gap.Data(:);
gapDes = P.acc.d0 + P.acc.Th * out.vEgo.Data(:);
vGap = out.vLead.Data(:) + P.acc.gap.P * (gap - gapDes);
following = gap < P.acc.range & vGap < P.acc.vSet;
end
