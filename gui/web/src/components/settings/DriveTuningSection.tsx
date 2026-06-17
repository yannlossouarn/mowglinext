import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
    Alert,
    Button,
    Card,
    Descriptions,
    Select,
    Space,
    Tag,
    Typography,
} from "antd";
import {
    ExperimentOutlined,
    PlayCircleOutlined,
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

type Status = Record<string, unknown>;

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
    const [maneuver, setManeuver] = useState<string>("yaw_hunt");
    const [status, setStatus] = useState<Status | null>(null);
    const [connected, setConnected] = useState(false);
    const cmdReady = useRef(false);

    const onStatus = useCallback((raw: string) => {
        const s = parseStatus(raw);
        if (s) {
            setConnected(true);
            setStatus(s);
        }
    }, []);

    const statusStream = useWS<string>(
        () => setConnected(false),
        () => undefined,
        onStatus,
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
        return () => {
            statusStream.stop();
            cmdStream.stop();
        };
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    const send = useCallback(
        (action: string) => {
            const command = action === "stop" ? { action } : { action, maneuver };
            cmdStream.sendJsonMessage({ data: JSON.stringify(command) });
        },
        [cmdStream, maneuver],
    );

    const busy = Boolean(status?.busy);
    const phase = (status?.phase as string) ?? "idle";
    const result = status?.result as Record<string, unknown> | undefined;
    // During an optimize sweep the live fields carry the candidate + best so far;
    // the final "done" result carries {best, best_score, history}.
    const best = (status?.best ?? result?.best) as Record<string, unknown> | undefined;
    const bestScore = (status?.best_score ?? result?.best_score) as number | undefined;
    const nextCombo = status?.next_combo as Record<string, unknown> | undefined;

    // Metrics to surface: prefer the finished single-run result, else the live status.
    const metrics = useMemo<Record<string, unknown>>(() => {
        const m = (result && !("best" in result) ? result : status) ?? {};
        return m as Record<string, unknown>;
    }, [result, status]);

    return (
        <div>
            <Alert
                type="warning"
                showIcon
                style={{ marginBottom: 16 }}
                message="This drives the robot autonomously"
                description="Runs real maneuvers (pivots, swaths, transits) to score and optimize the live drive params. Keep the area clear, the blade off, and the robot off the dock. Requires the drive_tuning_node to be running (launch with drive_tuning:=true, or run scripts/drive_tuning_node.py)."
            />

            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <ExperimentOutlined style={{ marginRight: 6 }} />
                            Maneuver
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            Run a single maneuver to score the current params, or optimize to
                            coordinate-descend the firmware/drive knobs (deadband, hold gain,
                            angular-rate gains) toward the lowest score for this maneuver.
                        </Paragraph>
                    </div>
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
                            disabled={busy}
                            onClick={() => send("run")}
                        >
                            Run once
                        </Button>
                        <Button
                            type="primary"
                            icon={<ThunderboltOutlined />}
                            disabled={busy}
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
                            {maneuver === "yaw_hunt" && (
                                <>
                                    <Descriptions.Item label="yaw peak (rad)">{num(metrics?.yaw_peak)}</Descriptions.Item>
                                    <Descriptions.Item label="yaw final (rad)">{num(metrics?.yaw_final)}</Descriptions.Item>
                                    <Descriptions.Item label="back-creep (rad)">{num(metrics?.backcreep)}</Descriptions.Item>
                                    <Descriptions.Item label="undershoot (rad)">{num(metrics?.undershoot)}</Descriptions.Item>
                                </>
                            )}
                            {maneuver === "transit_pose" && (
                                <>
                                    <Descriptions.Item label="xy error (m)">{num(metrics?.pose_xy_err)}</Descriptions.Item>
                                    <Descriptions.Item label="yaw error (rad)">{num(metrics?.pose_yaw_err)}</Descriptions.Item>
                                </>
                            )}
                            {(maneuver === "outline" || maneuver === "swath") && (
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
                        <Paragraph type="secondary" style={{ margin: 0 }}>
                            The optimizer leaves the robot on the best combo. To make it permanent,
                            copy the values into <Text strong>Drive Motor</Text> settings and save —
                            otherwise the firmware reverts to the saved values on the next reconnect.
                        </Paragraph>
                    </Space>
                )}
            </Card>
        </div>
    );
};
