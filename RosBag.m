TOP.odom   = "/odom";
TOP.wrench = "/steelhead/controls/input_forces";

CFG.dt = 0.02;

CFG.settle_band_yaw_deg = 2.0;
CFG.settle_band_ct_m    = 0.15;
CFG.settle_hold_s       = 1.0;

CFG.force_lat_limit   = NaN;
CFG.torque_yaw_limit  = NaN;

CFG.gate_xyz  = [-10, 0, 2];
CFG.start_xyz = [0, 0, 0.2];

bag = ros2bagreader("gate_run_01/");

odomSel = select(bag, "Topic", TOP.odom);
odomMsg = readMessages(odomSel);
[tO, pO, yawO] = extractOdom(odomMsg);

wrenchSel = select(bag, "Topic", TOP.wrench);
wrenchMsg = readMessages(wrenchSel;
[tU, uLat, uYaw] = extractWrench(wrenchMsg);

t0 = max([tO(1), tU(1)]);
t1 = min([tO(end), tU(end)]);
t  = (t0:CFG.dt:t1)';

pO_i   = interp1(tO, pO,   t, "linear", "extrap");
yawO_i = interp1(tO, yawO, t, "linear", "extrap");

uLat_i = interp1(tU, uLat, t, "linear", "extrap");
uYaw_i = interp1(tU, uYaw, t, "linear", "extrap");

start_xy = CFG.start_xyz(1:2);
gate_xy  = CFG.gate_xyz(1:2);

v = gate_xy - start_xy;
vn = norm(v);
tHat = v ./ max(vn, 1e-9);

dp = pO_i(:,1:2) - start_xy;

e_ct = dp(:,1).*tHat(2) - dp(:,2).*tHat(1);

yawDes = atan2(tHat(2), tHat(1));
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
    sat_yaw_pct = 100 * mean(abs(uYaw_i) >= 0.99 * CFG.torque_yaw_limit);
end
if ~isnan(CFG.force_lat_limit)
    sat_lat_pct = 100 * mean(abs(uLat_i) >= 0.99 * CFG.force_lat_limit);
end

out = struct();
out.t               = t;
out.cross_track_m   = e_ct;
out.heading_err_rad = e_yaw;
out.u_lat_force     = uLat_i;
out.u_yaw_torque    = uYaw_i;
out.yaw_des_rad     = yawDes;

out.metrics = table( ...
    ts_yaw, ts_ct, ...
    peak_yaw, peak_ct, ...
    eff_yaw, eff_lat, ...
    sat_yaw_pct, sat_lat_pct, ...
    'VariableNames', {'settle_yaw_s','settle_ct_s', ...
                      'peak_yaw_rad','peak_ct_m', ...
                      'eff_yaw_int_abs','eff_lat_int_abs', ...
                      'sat_yaw_pct','sat_lat_pct'} );

disp(out.metrics);

figure;
plot(t - t(1), e_ct); grid on;
xlabel("t (s)");
ylabel("cross-track error (m)");
title("Cross-track error vs time (start->gate line)");

figure;
plot(t - t(1), rad2deg(e_yaw)); grid on;
xlabel("t (s)");
ylabel("heading error (deg)");
title("Heading error vs time (constant desired yaw)");

figure;
plot(t - t(1), uLat_i); grid on;
xlabel("t (s)");
ylabel("lateral force command");
title("Control input: lateral (Wrench.force.y)");

figure;
plot(t - t(1), uYaw_i); grid on;
xlabel("t (s)");
ylabel("yaw torque command");
title("Control input: yaw (Wrench.torque.z)");


function [t, p, yaw] = extractOdom(msgs)
    n = numel(msgs);
    t = zeros(n,1);
    p = zeros(n,3);
    yaw = zeros(n,1);

    for k = 1:n
        t(k) = stampToSec(msgs{k}.header.stamp);

        pos = msgs{k}.pose.pose.position;
        p(k,1) = double(pos.x);
        p(k,2) = double(pos.y);
        p(k,3) = double(pos.z);

        q = msgs{k}.pose.pose.orientation;
        yaw(k) = quatYaw(double(q.w), double(q.x), double(q.y), double(q.z));
    end

    yaw = unwrap(yaw);
end

function [t, uLat, uYaw] = extractWrench(msgs)
    n = numel(msgs);
    t = zeros(n,1);
    uLat = zeros(n,1);
    uYaw = zeros(n,1);

    for k = 1:n
        if isfield(msgs{k}, "header") && isfield(msgs{k}.header, "stamp")
            t(k) = stampToSec(msgs{k}.header.stamp);
        elseif isfield(msgs{k}, "stamp")
            t(k) = stampToSec(msgs{k}.stamp);
        else
            t(k) = NaN;
        end

        f = msgs{k}.force;
        tau = msgs{k}.torque;

        uLat(k) = double(getField(f, "y", 0));
        uYaw(k) = double(getField(tau, "z", 0));
    end

    if any(isnan(t))
        t = (0:n-1)';
    end
end

function ts = stampToSec(stamp)
    ts = double(stamp.sec) + 1e-9 * double(stamp.nanosec);
end

function y = quatYaw(w, x, yq, z)
    sinY = 2*(w*z + x*yq);
    cosY = 1 - 2*(yq*yq + z*z);
    y = atan2(sinY, cosY);
end

function ts = settlingTime(t, e, band, hold_s)
    dt = median(diff(t));
    holdN = max(1, round(hold_s / max(dt, 1e-9)));
    inside = abs(e) <= band;
    ts = NaN;

    for k = 1:(numel(t)-holdN)
        if all(inside(k:k+holdN))
            ts = t(k) - t(1);
            return;
        end
    end
end

function v = getField(s, name, defaultVal)
    if isfield(s, name)
        v = s.(name);
    else
        v = defaultVal;
    end
end
