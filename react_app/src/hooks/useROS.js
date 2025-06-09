import { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';

const useROS = (rosbridgeUrl = 'ws://localhost:9090') => {
    const [connected, setConnected] = useState(false);
    const [error, setError] = useState(null);
    const rosRef = useRef(null);

    useEffect(() => {
        // Create ROS connection
        const ros = new ROSLIB.Ros({
            url: rosbridgeUrl
        });

        ros.on('connection', () => {
            console.log('Connected to ROS websocket server.');
            setConnected(true);
            setError(null);
        });

        ros.on('error', (error) => {
            console.log('Error connecting to ROS websocket server:', error);
            setError(error.toString());
            setConnected(false);
        });

        ros.on('close', () => {
            console.log('Connection to ROS websocket server closed.');
            setConnected(false);
        });

        rosRef.current = ros;

        // Cleanup on unmount
        return () => {
            if (rosRef.current) {
                rosRef.current.close();
            }
        };
    }, [rosbridgeUrl]);

    return {
        ros: rosRef.current,
        connected,
        error
    };
};

export default useROS; 