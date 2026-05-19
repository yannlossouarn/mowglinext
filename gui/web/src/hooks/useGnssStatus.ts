import {useEffect, useState} from "react";
import {GnssStatus} from "../types/ros.ts";
import {useWS} from "./useWS.ts";

export const useGnssStatus = () => {
    const [gnssStatus, setGnssStatus] = useState<GnssStatus>({});
    const gnssStatusStream = useWS<string>(() => {
            console.log({
                message: "GNSS status stream closed",
            });
        }, () => {
            console.log({
                message: "GNSS status stream connected",
            });
        },
        (e) => {
            setGnssStatus(JSON.parse(e));
        });
    useEffect(() => {
        gnssStatusStream.start("/api/mowglinext/subscribe/gnssStatus");
        return () => {
            gnssStatusStream.stop();
        };
    }, []);
    return gnssStatus;
};
