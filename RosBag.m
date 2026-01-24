function out = RosBag(bagFolder)

    TOP.odom   = "/odom";
    TOP.goal   = "/steelhead/controls/waypoint_marker/current_goal";
    TOP.goal2  = "/steelhead/controls/waypoint_marker/set";
    TOP.wrench = "/steelhead/controls/input_forces";

    CFG.dt = 0.02;

    CFG.settle_band_yaw_deg = 2.0;
    CFG.settle_band_ct_m    = 0.15;
    CFG.settle_hold_s       = 1.0;

    CFG.force_lat_limit   = NaN;
    CFG.torque_yaw_limit = NaN;

    bag = ros2bagreader(bagFolder);

    odomSel = select(bag, "Topic", TOP.odom);
    odomMsg = readMessages(odomSel);
    [tO, pO, yawO] = extractOdom(odomMsg);

    goalTopic = "";
    if any(strcmp(bag.AvailableTopics.Topic, TOP.goal))
        goalTopic = TOP.goal;
    elseif any(strcmp(bag.AvailableTopics.Topic, TOP.goal2))
        goalTopic = TOP.goal2;
    else
        error("No goal waypoint topic found. Expected %s or %s", ...
              TOP.goal, TOP.goal2);
    end

    goalSel = select(bag, "Topic", goalTopic);
    goalMsg = readMessages(goalSel, "DataFormat", "struct");
    [tG, pG] = extractWaypoint(goalMsg);

    wrenchSel = select(bag, "Topic", TOP.wrench);
    wrenchMsg = readMessages(wrenchSel, "DataFormat", "struct");
    [tU, uLat, uYaw] = extractWrench(wrenchMsg);

    t0 = max([tO(1),  tG(1),  tU(1)]);
    t1 = min([tO(end), tG(end), tU(end)]);
    t  = (t0:CFG.dt:t1)';

    pO_i   = interp1(tO, pO,   t, "linear",   "extrap");
    yawO_i = interp1(tO, yawO, t, "linear",   "extrap");
    pG_i   = interp1(tG, pG,   t, "previous", "extrap");

    uLat_i = interp1(tU, uLat, t, "linear", "extrap");
    uYaw_i = interp1(tU, uYaw, t, "linear", "extrap");

    v  = pG_i(:,1:2) - pO_i(:,1:2);
    vn = vecnorm(v, 2, 2);
    tHat = v ./ max(vn, 1e-9);

    dx = pO_i(:,1) - pG_i(:,1);
    dy = pO_i(:,2) - pG_i(:,2);

    e_ct = dx .* tHat(:,2) - dy .* tHat(:,1);

    yawDes = atan2(tHat(:,2), tHat(:,1));
    e_yaw  = wrapToPi(yawDes - yawO_i);

    bandYaw = deg2rad(CFG.settle_band_yaw_deg);
    bandCT  = CFG.settle_band_ct_m;

    ts_yaw = settlingTime(t, e_yaw, bandYaw, CFG.settle_hold_s);
    ts_ct  = settlingTime(t, e_ct,  bandCT,  CFG.settle_hold_s);

    peak_yaw = max(abs(e_yaw));
    peak_ct  = max(abs(e_ct));

    eff_yaw = trapz(t, abs(uYaw_i));
    eff_lat = trapz(t, abs(uLat_i));

    sat_yaw_pct = NaN;
    sat_lat_pct = NaN;

    if ~isnan(CFG.torque_yaw_limit)
        sat_yaw_pct = 100 * mean(abs(uYaw_i) >= ...
                        0.99 * CFG.torque_yaw_limit);
    end

    if ~isnan(CFG.force_lat_limit)
        sat_lat_pct = 100 * mean(abs(uLat_i) >= ...
                        0.99 * CFG.force_lat_limit);
    end

    out = struct();
    out.t               = t;
    out.cross_track_m   = e_ct;
    out.heading_err_rad = e_yaw;
    out.u_lat_force     = uLat_i;
    out.u_yaw_torque    = uYaw_i;

    out.metrics = table( ...
        ts_yaw, ts_ct, ...
        peak_yaw, peak_ct, ...
        eff_yaw, eff_lat, ...
        sat_yaw_pct, sat_lat_pct, ...
        'VariableNames', { ...
            "settle_yaw_s", "settle_ct_s", ...
            "peak_yaw_rad", "peak_ct_m", ...
            "eff_yaw_int_abs", "eff_lat_int_abs", ...
            "sat_yaw_pct", "sat_lat_pct" } );

    disp(out.metrics);

    figure;
    plot(t - t(1), e_ct); grid on;
    xlabel("t (s)");
    ylabel("cross-track error (m)");
    title("Cross-track error vs time");

    figure;
    plot(t - t(1), rad2deg(e_yaw)); grid on;
    xlabel("t (s)");
    ylabel("heading error (deg)");
    title("Heading error vs time");

    figure;
    plot(t - t(1), uLat_i); grid on;
    xlabel("t (s)");
    ylabel("lateral force command");
    title("Control input: lateral");

    figure;
    plot(t - t(1), uYaw_i); grid on;
    xlabel("t (s)");
    ylabel("yaw torque command");
    title("Control input: yaw");

end
