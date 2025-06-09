import { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';

const useROS = (rosbridgeUrl = 'ws://localhost:9090') => {
    const [connected, setConnected] = useState(false);
    const [error, setError] = useState(null);
    const rosRef = useRef(null);
    const reconnectTimeoutRef = useRef(null);

    const createConnection = () => {
        // Clear any existing reconnect timeout
        if (reconnectTimeoutRef.current) {
            clearTimeout(reconnectTimeoutRef.current);
        }

        // Create ROS connection
        const ros = new ROSLIB.Ros({
            url: rosbridgeUrl
        });

        ros.on('connection', () => {
            console.log('✅ Connected to ROS websocket server.');
            setConnected(true);
            setError(null);
        });

        ros.on('error', (error) => {
            console.error('❌ Error connecting to ROS websocket server:', error);

            // Handle different types of errors
            let errorMessage = 'Connection error';
            if (error && error.toString) {
                errorMessage = error.toString();
            } else if (typeof error === 'string') {
                errorMessage = error;
            }

            // Check if it's a JSON parsing error
            if (errorMessage.includes('JSON') || errorMessage.includes('parse')) {
                console.warn('🔧 JSON parsing error detected, this may be due to malformed ROS messages');
                errorMessage = 'JSON parsing error - check ROS message format';
            }

            setError(errorMessage);
            setConnected(false);

            // Don't auto-reconnect on JSON errors to avoid loops
            if (!errorMessage.includes('JSON') && !errorMessage.includes('Connection refused')) {
                console.log('🔄 Attempting to reconnect in 3 seconds...');
                reconnectTimeoutRef.current = setTimeout(() => {
                    console.log('🔌 Reconnecting to ROS...');
                    createConnection();
                }, 3000);
            }
        });

        ros.on('close', () => {
            console.log('🔌 Connection to ROS websocket server closed.');
            setConnected(false);

            // Attempt to reconnect after a delay
            console.log('🔄 Attempting to reconnect in 5 seconds...');
            reconnectTimeoutRef.current = setTimeout(() => {
                console.log('🔌 Reconnecting to ROS...');
                createConnection();
            }, 5000);
        });

        rosRef.current = ros;
    };

    useEffect(() => {
        createConnection();

        // Cleanup on unmount
        return () => {
            if (reconnectTimeoutRef.current) {
                clearTimeout(reconnectTimeoutRef.current);
            }
            if (rosRef.current) {
                try {
                    rosRef.current.close();
                } catch (closeError) {
                    console.warn('Error closing ROS connection:', closeError);
                }
            }
        };
    }, [rosbridgeUrl]);

    return {
        ros: rosRef.current,
        connected,
        error,
        reconnect: createConnection
    };
};

export default useROS; 