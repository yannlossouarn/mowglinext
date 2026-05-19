import {AbsolutePoseConstants, GnssStatus, GnssStatusConstants} from "../types/ros.ts";

export type GpsFixType = "RTK_FIX" | "RTK_FLOAT" | "GPS_FIX" | "NO_FIX";

export interface GpsStatus {
    fixType: GpsFixType;
    label: string;
    percent: number;
}

function fromFixType(fixType: number | undefined | null): GpsStatus | null {
    switch (fixType) {
        case GnssStatusConstants.FIX_TYPE_RTK_FIXED:
            return {fixType: "RTK_FIX", label: "RTK fixed", percent: 100};
        case GnssStatusConstants.FIX_TYPE_RTK_FLOAT:
            return {fixType: "RTK_FLOAT", label: "RTK float", percent: 50};
        case GnssStatusConstants.FIX_TYPE_GPS_FIX:
            return {fixType: "GPS_FIX", label: "GPS fix", percent: 25};
        case GnssStatusConstants.FIX_TYPE_DEAD_RECKONING:
            return {fixType: "NO_FIX", label: "Dead reckoning", percent: 10};
        case GnssStatusConstants.FIX_TYPE_NO_FIX:
            return {fixType: "NO_FIX", label: "No GPS", percent: 0};
        default:
            return null;
    }
}

function fromAbsolutePoseFlags(flags: number | undefined | null): GpsStatus {
    const f = flags ?? 0;
    if (f & AbsolutePoseConstants.FLAG_GPS_RTK_FIXED) {
        return {fixType: "RTK_FIX", label: "RTK fixed", percent: 100};
    }
    if (f & AbsolutePoseConstants.FLAG_GPS_RTK_FLOAT) {
        return {fixType: "RTK_FLOAT", label: "RTK float", percent: 50};
    }
    if (f & AbsolutePoseConstants.FLAG_GPS_RTK) {
        return {fixType: "GPS_FIX", label: "GPS fix", percent: 25};
    }
    return {fixType: "NO_FIX", label: "No GPS", percent: 0};
}

// Source of truth: GnssStatus from /gps/status. Fall back to the legacy
// AbsolutePose flags while older publishers are still around.
export function deriveGpsStatus(
    gnssStatus: GnssStatus | undefined | null,
    fallbackFlags?: number | undefined | null,
): GpsStatus {
    const fromTypedStatus = fromFixType(gnssStatus?.fix_type);
    if (fromTypedStatus) {
        return fromTypedStatus;
    }
    return fromAbsolutePoseFlags(fallbackFlags);
}
