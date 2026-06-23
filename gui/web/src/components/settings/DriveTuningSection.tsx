import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
    Alert,
    Button,
    Card,
    Checkbox,
    Descriptions,
    Divider,
    Segmented,
    Select,
    Space,
    Steps,
    Table,
    Tag,
    Typography,
} from "antd";
import {
    ExperimentOutlined,
    PlayCircleOutlined,
    SaveOutlined,
    StopOutlined,
    ThunderboltOutlined,
} from "@ant-design/icons";
import { useWS } from "../../hooks/useWS.ts";

const { Text, Paragraph } = Typography;

// Maneuvers exposed by the drive_tuning_node (see ros2/scripts/drive_tuning_node.py).
const MANEUVERS = [
    { value: "yaw_hunt", label: "In-place yaw hunt (small pivot)" },
    { value: "outline", label: "Outline (closed diamond, sharp corners)" },
    { value: "swath", label: "Coverage swath (plan_coverage)" },
    { value: "transit_pose", label: "Transit to precise pose (docking sim)" },
];

const PUB_URI = "/api/mowglinext/publish/driveTuningCommand";
const SUB_URI = "/api/mowglinext/subscribe/driveTuningStatus";
const TELEM_SUB_URI = "/api/mowglinext/subscribe/driveTelemetry";

type Status = Record<string, unknown>;

// One step of the guided protocol, published by the node on `get_protocol`
// (see PROTOCOL in ros2/scripts/drive_tuning_node.py).
type ProtocolStep = {
    id: string;
    title: string;
    params: string[];
    maneuver: string;
    clearance: string;
    guidance: string;
    optional?: boolean;
    requires_pi?: boolean;
    pi_run?: string; // "off"|"on": the loop mode the tuner forces for this step
};

// /hardware_bridge/drive_telemetry Float32MultiArray layout (hardware_bridge_node.cpp).
type Telem = {
    lTgt: number; rTgt: number; lAct: number; rAct: number;
    lPwm: number; rPwm: number; wheelYaw: number; imuYaw: number;
    resid: number; accelPeakG: number; lLoad: number; rLoad: number; flags: number;
};

// slip_flags bits — see DRIVE_SLIP_FLAG_* in mowgli_protocol.h.
const SLIP_FLAGS: { mask: number; label: string; color: string }[] = [
    { mask: 1 << 0, label: "YAW", color: "blue" },
    { mask: 1 << 1, label: "STALL", color: "orange" },
    { mask: 1 << 2, label: "IMPACT", color: "red" },
    { mask: 1 << 3, label: "BOG", color: "gold" },
    { mask: 1 << 4, label: "BLADE_BOG", color: "volcano" },
    { mask: 1 << 5, label: "JAM", color: "red" },
];

function parseTelem(raw: string): Telem | null {
    try {
        const obj = JSON.parse(raw) as { data?: number[] };
        const d = obj?.data;
        if (!Array.isArray(d) || d.length < 13) return null;
        return {
            lTgt: d[0], rTgt: d[1], lAct: d[2], rAct: d[3], lPwm: d[4], rPwm: d[5],
            wheelYaw: d[6], imuYaw: d[7], resid: d[8], accelPeakG: d[9],
            lLoad: d[10], rLoad: d[11], flags: d[12],
        };
    } catch {
        return null;
    }
}

// std_msgs/String wraps the node's JSON in a `data` field; unwrap then parse.
function parseStatus(raw: string): Status | null {
    try {
        const outer = JSON.parse(raw) as { data?: string };
        const inner = typeof outer?.data === "string" ? outer.data : raw;
        return JSON.parse(inner) as Status;
    } catch {
        return null;
    }
}

function num(v: unknown, digits = 3): string {
    return typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "—";
}

// --- Tunable-parameter panel (status.params from the node) -------------------
// One entry per tunable: live value, the saved (mowgli_robot.yaml) value, the
// session-start baseline, and whether it's absent from the YAML (firmware default).
type ParamCell = {
    live: number | boolean;
    saved: number | boolean | null;
    default: boolean;
    baseline: number | boolean | null;
};

// Friendly labels + render order. Mirrors the protocol step order.
const PARAM_LABELS: Record<string, string> = {
    wheel_pid_deadband_pwm: "Breakaway deadband (PWM)",
    wheel_pid_pwm_per_mps: "Speed scale (PWM per m/s)",
    wheel_pid_kp: "Speed PI — Kp",
    wheel_pid_ki: "Speed PI — Ki",
    wheel_pid_integral_limit: "Speed PI — integral limit",
    wheel_hold_kp: "Standstill hold — Kp",
    angular_rate_kp: "Turn rate — Kp",
    angular_rate_ki: "Turn rate — Ki",
    wheel_pi_enabled: "Closed-loop wheel PI",
};
const PARAM_ORDER = Object.keys(PARAM_LABELS);

function fmtVal(v: number | boolean | null | undefined): string {
    if (typeof v === "boolean") return v ? "on" : "off";
    if (typeof v === "number" && Number.isFinite(v)) return v.toFixed(2);
    return "—";
}

function approxEq(a: unknown, b: unknown): boolean {
    if (typeof a === "boolean" || typeof b === "boolean") return a === b;
    if (typeof a === "number" && typeof b === "number") return Math.abs(a - b) < 1e-4;
    return false;
}

// State badge: "saved" (matches the persisted config), "default" (firmware
// default, untouched + not in the YAML), or "overridden" (a test changed it
// and/or it differs from the saved config → not yet persisted).
function paramState(c: ParamCell): { text: string; color: string } {
    const matchesSaved = !c.default && approxEq(c.live, c.saved);
    const changed = c.baseline != null && !approxEq(c.live, c.baseline);
    if (matchesSaved) return { text: "saved", color: "green" };
    if (c.default && !changed) return { text: "default", color: "blue" };
    return { text: "overridden", color: "orange" };
}

type ParamRow = {
    key: string;
    name: string;
    current: string;
    saved: string;
    state: { text: string; color: string };
};

const PARAM_COLUMNS = [
    { title: "Parameter", dataIndex: "name", key: "name" },
    { title: "Current", dataIndex: "current", key: "current", align: "right" as const },
    { title: "Saved", dataIndex: "saved", key: "saved", align: "right" as const },
    {
        title: "State",
        key: "state",
        align: "center" as const,
        render: (_: unknown, r: ParamRow) => <Tag color={r.state.color}>{r.state.text}</Tag>,
    },
];

// Render an arbitrary {param: value} combo as compact tags.
const Combo: React.FC<{ combo?: Record<string, unknown> }> = ({ combo }) => {
    if (!combo || typeof combo !== "object") return <Text type="secondary">—</Text>;
    return (
        <Space size={[4, 4]} wrap>
            {Object.entries(combo).map(([k, v]) => (
                <Tag key={k}>{`${k}=${typeof v === "number" ? num(v, 2) : String(v)}`}</Tag>
            ))}
        </Space>
    );
};

export const DriveTuningSection: React.FC = () => {
    const [mode, setMode] = useState<"guided" | "advanced">("guided");
    const [maneuver, setManeuver] = useState<string>("yaw_hunt");
    const [protocol, setProtocol] = useState<ProtocolStep[]>([]);
    const [notes, setNotes] = useState<string>("");
    const [current, setCurrent] = useState(0);
    const [status, setStatus] = useState<Status | null>(null);
    const [connected, setConnected] = useState(false);
    const cmdReady = useRef(false);

    const onStatus = useCallback((raw: string) => {
        const s = parseStatus(raw);
        if (!s) return;
        // The node answers `get_protocol` with a {protocol:[...], notes:"..."} message.
        if (Array.isArray((s as { protocol?: unknown }).protocol)) {
            setProtocol((s as { protocol: ProtocolStep[] }).protocol);
            const n = (s as { notes?: unknown }).notes;
            if (typeof n === "string") setNotes(n);
            return;
        }
        setConnected(true);
        setStatus(s);
    }, []);

    const [telem, setTelem] = useState<Telem | null>(null);
    const onTelem = useCallback((raw: string) => {
        const t = parseTelem(raw);
        if (t) setTelem(t);
    }, []);

    const statusStream = useWS<string>(
        () => setConnected(false),
        () => undefined,
        onStatus,
    );
    const telemStream = useWS<string>(
        () => undefined,
        () => undefined,
        onTelem,
    );
    const cmdStream = useWS<string>(
        () => {
            cmdReady.current = false;
        },
        () => {
            cmdReady.current = true;
        },
        () => undefined,
    );

    // Start both streams on mount; tear down on unmount.
    useEffect(() => {
        statusStream.start(SUB_URI);
        cmdStream.start(PUB_URI);
        telemStream.start(TELEM_SUB_URI);
        return () => {
            statusStream.stop();
            cmdStream.stop();
            telemStream.stop();
        };
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    // Pull the protocol once the command channel is up; retry until it arrives.
    useEffect(() => {
        if (protocol.length > 0) return;
        const id = setInterval(() => {
            cmdStream.sendJsonMessage({ data: JSON.stringify({ action: "get_protocol" }) });
        }, 1500);
        return () => clearInterval(id);
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [protocol.length]);

    const send = useCallback(
        (action: string, step?: string) => {
            let command: Record<string, unknown>;
            if (
                action === "stop" ||
                action === "start_session" ||
                action === "stop_session" ||
                action === "persist"
            )
                command = { action };
            else if (step) command = { action, step };
            else command = { action, maneuver };
            cmdStream.sendJsonMessage({ data: JSON.stringify(command) });
        },
        [cmdStream, maneuver],
    );

    const busy = Boolean(status?.busy);
    const armed = Boolean(status?.armed);
    // Live /hardware_bridge wheel_pi_enabled (undefined until the node reports it).
    const wheelPiEnabled = status?.wheel_pi_enabled as boolean | undefined;
    const phase = (status?.phase as string) ?? "idle";
    // Per-step clearance confirmation — must be re-affirmed whenever the active
    // step (guided) or maneuver/mode (advanced) changes, since each runs an
    // autonomous descent that moves the robot.
    const [confirmed, setConfirmed] = useState(false);
    useEffect(() => {
        setConfirmed(false);
    }, [current, maneuver, mode]);
    const result = status?.result as Record<string, unknown> | undefined;
    // During an optimize sweep the live fields carry the candidate + best so far;
    // the final "done" result carries {best, best_score, history}.
    const best = (status?.best ?? result?.best) as Record<string, unknown> | undefined;
    const bestScore = (status?.best_score ?? result?.best_score) as number | undefined;
    const nextCombo = status?.next_combo as Record<string, unknown> | undefined;
    // Result of the last "persist" (Save tuned values to config) action.
    const persist = status?.persist as
        | { persisted?: boolean; written?: Record<string, unknown>; error?: string }
        | undefined;

    // Live tunable-parameter panel (current value of each param + default/overridden state).
    const params = status?.params as Record<string, ParamCell> | undefined;
    const paramRows = useMemo<ParamRow[]>(() => {
        if (!params) return [];
        return PARAM_ORDER.filter((k) => k in params).map((k) => {
            const c = params[k];
            return {
                key: k,
                name: PARAM_LABELS[k],
                current: fmtVal(c.live),
                saved: c.default ? "—" : fmtVal(c.saved),
                state: paramState(c),
            };
        });
    }, [params]);
    const overriddenCount = paramRows.filter((r) => r.state.text === "overridden").length;

    // Metrics to surface: prefer the finished single-run result, else the live status.
    const metrics = useMemo<Record<string, unknown>>(() => {
        const m = (result && !("best" in result) ? result : status) ?? {};
        return m as Record<string, unknown>;
    }, [result, status]);

    const activeFlags = telem ? SLIP_FLAGS.filter((f) => (telem.flags & f.mask) !== 0) : [];

    // In guided mode the active maneuver is the current step's; advanced mode
    // uses the dropdown selection. Metrics/labels key off this.
    const step = mode === "guided" ? protocol[current] : undefined;
    const activeManeuver = step?.maneuver ?? maneuver;
    // A PI-dependent step is blocked when closed-loop PI is known to be off.
    const piBlocked = Boolean(step?.requires_pi) && wheelPiEnabled === false;
    let piTagColor = "default";
    let piTagText = "unknown";
    if (wheelPiEnabled === true) {
        piTagColor = "green";
        piTagText = "on";
    } else if (wheelPiEnabled === false) {
        piTagColor = "red";
        piTagText = "off";
    }

    return (
        <div>
            <Alert
                type="warning"
                showIcon
                style={{ marginBottom: 16 }}
                message="This drives the robot autonomously"
                description="Runs real maneuvers (pivots, swaths, transits) to score and optimize the live drive params. Keep the area clear, the blade off, and the robot off the dock. Requires the drive_tuning_node to be running (launch with drive_tuning:=true, or run scripts/drive_tuning_node.py)."
            />

            <Segmented
                block
                style={{ marginBottom: 16 }}
                value={mode}
                onChange={(v) => setMode(v as "guided" | "advanced")}
                options={[
                    { label: "Guided protocol", value: "guided" },
                    { label: "Advanced (single maneuver)", value: "advanced" },
                ]}
            />

            <Card size="small" style={{ marginBottom: 16 }}>
                <Space wrap align="center">
                    {armed ? (
                        <Button danger icon={<StopOutlined />} onClick={() => send("stop_session")}>
                            Stop tuning
                        </Button>
                    ) : (
                        <Button
                            type="primary"
                            icon={<PlayCircleOutlined />}
                            onClick={() => send("start_session")}
                        >
                            Start tuning
                        </Button>
                    )}
                    <Tag color={armed ? "processing" : "default"}>
                        {armed ? "session active" : "idle"}
                    </Tag>
                    <Tag color={piTagColor}>{`closed-loop PI: ${piTagText}`}</Tag>
                    <Text type="secondary">
                        Start activates the live odom/IMU/cmd_vel subscriptions; stop releases
                        them. Maneuvers run only while a session is active. "closed-loop PI" is
                        your operating mode (the tuner still switches per step).
                    </Text>
                </Space>
            </Card>

            {mode === "guided" ? (
                <Card size="small" style={{ marginBottom: 16 }} title="Guided drive tuning">
                    {protocol.length === 0 ? (
                        <Text type="secondary">
                            Waiting for the protocol from <code>/drive_tuning_node</code> — start the
                            node if this persists.
                        </Text>
                    ) : (
                        <Space direction="vertical" size={12} style={{ width: "100%" }}>
                            <Paragraph type="secondary" style={{ margin: 0 }}>
                                Tune the drive in order — each step builds on the ones above it. Read
                                the placement note, position the robot, confirm the area is clear,
                                then Optimize. During a step the robot returns to its start spot
                                between tries, so it needs only the listed clearance forward plus
                                room to pivot in place — it will not march further away. Move on
                                when the score stops improving. The early steps run open-loop to
                                identify the drivetrain and the PI steps run closed-loop; the tuner
                                sets that per step and restores your operating mode afterwards.
                            </Paragraph>
                            {notes && (
                                <Alert type="info" showIcon message="Note" description={notes} style={{ margin: 0 }} />
                            )}
                            <Steps
                                current={current}
                                onChange={setCurrent}
                                direction="vertical"
                                size="small"
                                items={protocol.map((s) => ({
                                    title: s.title,
                                    description: s.clearance,
                                }))}
                            />
                            {step && (
                                <Card size="small" type="inner" title={step.title}>
                                    <Space direction="vertical" size={10} style={{ width: "100%" }}>
                                        <Paragraph style={{ margin: 0 }}>{step.guidance}</Paragraph>
                                        <Alert
                                            type="info"
                                            showIcon
                                            message={`Place the robot: ${step.clearance}`}
                                        />
                                        <Space size={[4, 4]} wrap>
                                            <Text type="secondary">Tunes:</Text>
                                            {step.params.map((p) => (
                                                <Tag key={p}>{p}</Tag>
                                            ))}
                                            {step.optional && <Tag color="default">optional</Tag>}
                                            {step.requires_pi && (
                                                <Tag color={piTagColor}>{`closed-loop PI: ${piTagText}`}</Tag>
                                            )}
                                            {step.pi_run && (
                                                <Tag color={step.pi_run === "off" ? "blue" : "geekblue"}>
                                                    {`tuner runs this ${step.pi_run === "off" ? "open-loop" : "closed-loop"}`}
                                                </Tag>
                                            )}
                                        </Space>
                                        {piBlocked && (
                                            <Alert
                                                type="warning"
                                                showIcon
                                                message="Closed-loop PI is off (wheel_pi_enabled)"
                                                description="This step tunes the closed-loop speed PI, which is currently disabled — running it now would have no effect. Enable PI in Drive Motor settings first, or skip this step."
                                            />
                                        )}
                                        <Checkbox
                                            checked={confirmed}
                                            disabled={busy}
                                            onChange={(e) => setConfirmed(e.target.checked)}
                                        >
                                            Robot is positioned and the area is clear ({step.clearance};
                                            it returns to this spot between tries).
                                        </Checkbox>
                                        <Space wrap>
                                            <Button
                                                icon={<PlayCircleOutlined />}
                                                disabled={busy || !armed || !confirmed || piBlocked}
                                                onClick={() => send("run_step", step.id)}
                                            >
                                                Test once
                                            </Button>
                                            <Button
                                                type="primary"
                                                icon={<ThunderboltOutlined />}
                                                disabled={busy || !armed || !confirmed || piBlocked}
                                                onClick={() => send("optimize_step", step.id)}
                                            >
                                                Optimize this step
                                            </Button>
                                            <Button
                                                danger
                                                icon={<StopOutlined />}
                                                disabled={!busy}
                                                onClick={() => send("stop")}
                                            >
                                                Stop
                                            </Button>
                                        </Space>
                                        <Divider style={{ margin: "4px 0" }} />
                                        <Space>
                                            <Button
                                                disabled={current === 0 || busy}
                                                onClick={() => setCurrent((c) => Math.max(0, c - 1))}
                                            >
                                                ← Previous
                                            </Button>
                                            <Button
                                                disabled={current >= protocol.length - 1 || busy}
                                                onClick={() =>
                                                    setCurrent((c) =>
                                                        Math.min(protocol.length - 1, c + 1),
                                                    )
                                                }
                                            >
                                                Next step →
                                            </Button>
                                        </Space>
                                    </Space>
                                </Card>
                            )}
                        </Space>
                    )}
                </Card>
            ) : (
                <Card size="small" style={{ marginBottom: 16 }}>
                    <Space direction="vertical" size={12} style={{ width: "100%" }}>
                        <div>
                            <Text strong style={{ fontSize: 14 }}>
                                <ExperimentOutlined style={{ marginRight: 6 }} />
                                Maneuver
                            </Text>
                            <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                                Run a single maneuver to score the current params, or optimize to
                                coordinate-descend the firmware/drive knobs (deadband, viscous, PI,
                                hold, angular-rate gains) toward the lowest score for this maneuver.
                            </Paragraph>
                        </div>
                        <Checkbox
                            checked={confirmed}
                            disabled={busy}
                            onChange={(e) => setConfirmed(e.target.checked)}
                        >
                            Robot is positioned and the area is clear for this maneuver (it returns
                            to its start spot between tries).
                        </Checkbox>
                        <Space wrap>
                            <Select
                                value={maneuver}
                                onChange={setManeuver}
                                options={MANEUVERS}
                                style={{ minWidth: 320 }}
                                disabled={busy}
                            />
                            <Button
                                type="default"
                                icon={<PlayCircleOutlined />}
                                disabled={busy || !armed || !confirmed}
                                onClick={() => send("run")}
                            >
                                Run once
                            </Button>
                            <Button
                                type="primary"
                                icon={<ThunderboltOutlined />}
                                disabled={busy || !armed || !confirmed}
                                onClick={() => send("optimize")}
                            >
                                Optimize
                            </Button>
                            <Button danger icon={<StopOutlined />} disabled={!busy} onClick={() => send("stop")}>
                                Stop
                            </Button>
                        </Space>
                    </Space>
                </Card>
            )}

            <Card
                size="small"
                style={{ marginBottom: 16 }}
                title={
                    <Space>
                        <span>Tunable parameters</span>
                        {overriddenCount > 0 && (
                            <Tag color="orange">{`${overriddenCount} unsaved`}</Tag>
                        )}
                    </Space>
                }
            >
                {paramRows.length === 0 ? (
                    <Text type="secondary">
                        Waiting for live parameter values from <code>/drive_tuning_node</code> — start
                        the node / session if this persists.
                    </Text>
                ) : (
                    <Space direction="vertical" size={8} style={{ width: "100%" }}>
                        <Table<ParamRow>
                            size="small"
                            pagination={false}
                            rowKey="key"
                            dataSource={paramRows}
                            columns={PARAM_COLUMNS}
                        />
                        <Text type="secondary">
                            <Tag color="green">saved</Tag> matches the saved config ·{" "}
                            <Tag color="blue">default</Tag> firmware default (not in config) ·{" "}
                            <Tag color="orange">overridden</Tag> changed by a test, not yet saved.
                            {overriddenCount > 0 &&
                                " Use “Save tuned values to config” in Status to persist."}
                        </Text>
                    </Space>
                )}
            </Card>

            <Card
                size="small"
                style={{ marginBottom: 16 }}
                title={
                    <Space>
                        <span>Live telemetry</span>
                        {telem ? <Tag color="success">streaming</Tag> : <Tag>no data</Tag>}
                    </Space>
                }
            >
                {!telem ? (
                    <Text type="secondary">
                        Waiting for <code>/hardware_bridge/drive_telemetry</code> — drive the robot
                        to see load, residual, and discrepancy flags.
                    </Text>
                ) : (
                    <Space direction="vertical" size={8} style={{ width: "100%" }}>
                        <Descriptions size="small" column={{ xs: 1, sm: 2, md: 3 }} bordered>
                            <Descriptions.Item label="load L/R">
                                {`${num(telem.lLoad, 0)} / ${num(telem.rLoad, 0)}`}
                            </Descriptions.Item>
                            <Descriptions.Item label="wheel−IMU resid (rad/s)">
                                {num(telem.resid, 2)}
                            </Descriptions.Item>
                            <Descriptions.Item label="accel peak (g)">
                                {num(telem.accelPeakG, 2)}
                            </Descriptions.Item>
                            <Descriptions.Item label="vel L/R (m/s)">
                                {`${num(telem.lAct, 2)} / ${num(telem.rAct, 2)}`}
                            </Descriptions.Item>
                            <Descriptions.Item label="target L/R (m/s)">
                                {`${num(telem.lTgt, 2)} / ${num(telem.rTgt, 2)}`}
                            </Descriptions.Item>
                            <Descriptions.Item label="PWM L/R">
                                {`${num(telem.lPwm, 0)} / ${num(telem.rPwm, 0)}`}
                            </Descriptions.Item>
                        </Descriptions>
                        <div>
                            <Text strong>Flags</Text>
                            <div style={{ marginTop: 4 }}>
                                {activeFlags.length > 0 ? (
                                    <Space size={[4, 4]} wrap>
                                        {activeFlags.map((f) => (
                                            <Tag key={f.label} color={f.color}>
                                                {f.label}
                                            </Tag>
                                        ))}
                                    </Space>
                                ) : (
                                    <Tag color="default">none</Tag>
                                )}
                            </div>
                        </div>
                    </Space>
                )}
            </Card>

            <Card
                size="small"
                title={
                    <Space>
                        <span>Status</span>
                        {connected ? (
                            <Tag color={busy ? "processing" : "success"}>{busy ? "running" : "ready"}</Tag>
                        ) : (
                            <Tag color="default">node not connected</Tag>
                        )}
                    </Space>
                }
            >
                {!connected ? (
                    <Text type="secondary">
                        Waiting for <code>/drive_tuning_node/status</code> — start the node to begin.
                    </Text>
                ) : (
                    <Space direction="vertical" size={12} style={{ width: "100%" }}>
                        <Descriptions size="small" column={{ xs: 1, sm: 2 }} bordered>
                            <Descriptions.Item label="Phase">{phase}</Descriptions.Item>
                            <Descriptions.Item label="BT state">
                                {String(status?.hl_state ?? "—")}
                            </Descriptions.Item>
                            <Descriptions.Item label="σ_xy (m)">
                                {num(status?.sigma_xy, 3)}
                            </Descriptions.Item>
                            <Descriptions.Item label="Score">
                                {num(metrics?.score ?? bestScore, 4)}
                            </Descriptions.Item>
                        </Descriptions>

                        {typeof metrics?.error === "string" && (
                            <Alert type="error" showIcon message={String(metrics.error)} />
                        )}

                        <Descriptions size="small" column={{ xs: 1, sm: 2 }} title="Last metrics">
                            {activeManeuver === "yaw_hunt" && (
                                <>
                                    <Descriptions.Item label="yaw peak (rad)">{num(metrics?.yaw_peak)}</Descriptions.Item>
                                    <Descriptions.Item label="yaw final (rad)">{num(metrics?.yaw_final)}</Descriptions.Item>
                                    <Descriptions.Item label="back-creep (rad)">{num(metrics?.backcreep)}</Descriptions.Item>
                                    <Descriptions.Item label="undershoot (rad)">{num(metrics?.undershoot)}</Descriptions.Item>
                                </>
                            )}
                            {activeManeuver === "transit_pose" && (
                                <>
                                    <Descriptions.Item label="xy error (m)">{num(metrics?.pose_xy_err)}</Descriptions.Item>
                                    <Descriptions.Item label="yaw error (rad)">{num(metrics?.pose_yaw_err)}</Descriptions.Item>
                                </>
                            )}
                            {(activeManeuver === "outline" || activeManeuver === "swath") && (
                                <>
                                    <Descriptions.Item label="cross-track RMS (m)">{num(metrics?.ct_rms)}</Descriptions.Item>
                                    <Descriptions.Item label="cross-track peak (m)">{num(metrics?.ct_peak)}</Descriptions.Item>
                                    <Descriptions.Item label="heading RMS (rad)">{num(metrics?.herr_rms)}</Descriptions.Item>
                                    <Descriptions.Item label="cmd-wz zero-cross (/s)">{num(metrics?.wz_zc, 2)}</Descriptions.Item>
                                </>
                            )}
                        </Descriptions>

                        <div>
                            <Text strong>Suggested next combo</Text>
                            <div style={{ marginTop: 4 }}>
                                <Combo combo={nextCombo} />
                            </div>
                        </div>
                        <div>
                            <Text strong>Best so far</Text>
                            <div style={{ marginTop: 4 }}>
                                <Combo combo={best} />
                                {typeof bestScore === "number" && (
                                    <Text type="secondary" style={{ marginLeft: 8 }}>
                                        (score {num(bestScore, 4)})
                                    </Text>
                                )}
                            </div>
                        </div>
                        <Divider style={{ margin: "4px 0" }} />
                        <Space direction="vertical" size={6} style={{ width: "100%" }}>
                            <Space wrap align="center">
                                <Button
                                    icon={<SaveOutlined />}
                                    disabled={busy}
                                    onClick={() => send("persist")}
                                >
                                    Save tuned values to config
                                </Button>
                                <Text type="secondary">
                                    Writes the live drive params to <code>mowgli_robot.yaml</code> so
                                    they survive a restart (otherwise the firmware reverts to the saved
                                    values on the next reconnect). Save after each step you're happy with.
                                </Text>
                            </Space>
                            {persist && (
                                <Alert
                                    type={persist.persisted ? "success" : "error"}
                                    showIcon
                                    message={
                                        persist.persisted
                                            ? "Saved to mowgli_robot.yaml"
                                            : "Save failed"
                                    }
                                    description={
                                        persist.persisted ? (
                                            <Combo combo={persist.written} />
                                        ) : (
                                            persist.error ?? "unknown error"
                                        )
                                    }
                                />
                            )}
                        </Space>
                    </Space>
                )}
            </Card>
        </div>
    );
};
