import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Space, Typography } from "antd";
import { DashboardOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

// Firmware per-wheel velocity PID + feedforward. Ranges match the clamps the
// STM32 applies on receipt (cpp_main.cpp on_set_drive_pid) and the schema.
export const DriveMotorSection: React.FC<Props> = ({ values, onChange }) => {
    const { t } = useTranslation();
    return (
        <div>
            <Alert
                type="info"
                showIcon
                style={{ marginBottom: 16 }}
                message={t("settingsDriveMotor.savedLiveTitle")}
                description={t("settingsDriveMotor.savedLiveDescription")}
            />
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <DashboardOutlined style={{ marginRight: 6 }} />
                            {t("settingsDriveMotor.wheelVelocityPid")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsDriveMotor.wheelVelocityPidDescription")}
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Kp" tooltip={t("settingsDriveMotor.kpTooltip")}>
                                    <InputNumber
                                        value={values.wheel_pid_kp}
                                        onChange={(v) => onChange("wheel_pid_kp", v)}
                                        min={0} max={200} step={1} precision={2}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Ki" tooltip={t("settingsDriveMotor.kiTooltip")}>
                                    <InputNumber
                                        value={values.wheel_pid_ki}
                                        onChange={(v) => onChange("wheel_pid_ki", v)}
                                        min={0} max={20000} step={100} precision={0}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Kd" tooltip={t("settingsDriveMotor.kdTooltip")}>
                                    <InputNumber
                                        value={values.wheel_pid_kd}
                                        onChange={(v) => onChange("wheel_pid_kd", v)}
                                        min={0} max={500} step={1} precision={2}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("settingsDriveMotor.integralLimit")} tooltip={t("settingsDriveMotor.integralLimitTooltip")}>
                                    <InputNumber
                                        value={values.wheel_pid_integral_limit}
                                        onChange={(v) => onChange("wheel_pid_integral_limit", v)}
                                        min={0} max={255} step={5} precision={0}
                                        style={{ width: "100%" }} addonAfter="PWM"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>

            <Card size="small" title={t("settingsDriveMotor.feedforward")} style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label={t("settingsDriveMotor.pwmPerMps")}
                                tooltip={t("settingsDriveMotor.pwmPerMpsTooltip")}
                            >
                                <InputNumber
                                    value={values.wheel_pid_pwm_per_mps}
                                    onChange={(v) => onChange("wheel_pid_pwm_per_mps", v)}
                                    min={50} max={600} step={10} precision={0}
                                    style={{ width: "100%" }} addonAfter="PWM"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
