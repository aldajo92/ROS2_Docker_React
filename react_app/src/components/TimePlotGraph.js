import React, { useState, useEffect, useRef } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import ROSLIB from 'roslib';
import './TimePlotGraph.css';

const TimePlotGraph = ({ title = "Time Plot", ros, connected }) => {
    const [data, setData] = useState([]);
    const [selectedTopic, setSelectedTopic] = useState('');
    const [isRecording, setIsRecording] = useState(false);
    const [messageType, setMessageType] = useState('');
    const startTimeRef = useRef(Date.now());
    const subscriberRef = useRef(null);

    // Topic configurations with their message types and data extraction paths
    const plottableTopics = [
        {
            name: '/sensor_data/temperature',
            type: 'std_msgs/Float64',
            path: 'data',
            color: '#ff7300'
        },
        {
            name: '/sensor_data/humidity',
            type: 'std_msgs/Float64',
            path: 'data',
            color: '#387908'
        },
        {
            name: '/sensor_data/battery_voltage',
            type: 'std_msgs/Float64',
            path: 'data',
            color: '#8884d8'
        },
        {
            name: '/sensor_data/pressure',
            type: 'std_msgs/Float64',
            path: 'data',
            color: '#82ca9d'
        },
        {
            name: '/odom/pose/pose/position/x',
            type: 'std_msgs/Float64',
            path: 'data',
            color: '#ffc658'
        },
        {
            name: '/odom/pose/pose/position/y',
            type: 'std_msgs/Float64',
            path: 'data',
            color: '#ff7c7c'
        },
        {
            name: '/cmd_vel',
            type: 'geometry_msgs/Twist',
            path: 'linear.x',
            color: '#8dd1e1'
        }
    ];

    // Window size in seconds (similar to Python example)
    const WINDOW_SIZE = 5.0;

    // Clear data
    const clearData = () => {
        setData([]);
        startTimeRef.current = Date.now();
    };

    // Extract value from message based on path
    const extractValue = (message, path) => {
        const parts = path.split('.');
        let value = message;
        for (const part of parts) {
            if (value && typeof value === 'object' && part in value) {
                value = value[part];
            } else {
                return 0; // Default value if path doesn't exist
            }
        }
        return typeof value === 'number' ? value : 0;
    };

    // Setup ROS subscription
    useEffect(() => {
        if (isRecording && selectedTopic && ros && connected) {
            const topicConfig = plottableTopics.find(t => t.name === selectedTopic);
            if (!topicConfig) return;

            console.log(`Subscribing to ${selectedTopic} with type ${topicConfig.type}`);

            // Create ROS subscriber
            const subscriber = new ROSLIB.Topic({
                ros: ros,
                name: selectedTopic,
                messageType: topicConfig.type
            });

            subscriber.subscribe((message) => {
                const currentTime = (Date.now() - startTimeRef.current) / 1000; // Time in seconds
                const value = extractValue(message, topicConfig.path);

                const newPoint = {
                    time: currentTime,
                    value: value,
                    timestamp: Date.now()
                };

                setData(prevData => {
                    const updatedData = [...prevData, newPoint];
                    // Keep only data within the time window
                    return updatedData.filter(point =>
                        (currentTime - point.time) <= WINDOW_SIZE
                    );
                });
            });

            subscriberRef.current = subscriber;
            setMessageType(topicConfig.type);

            // Cleanup function
            return () => {
                if (subscriberRef.current) {
                    subscriberRef.current.unsubscribe();
                    subscriberRef.current = null;
                }
            };
        }
    }, [isRecording, selectedTopic, ros, connected]);

    // Toggle recording
    const toggleRecording = () => {
        if (isRecording) {
            // Stop recording
            if (subscriberRef.current) {
                subscriberRef.current.unsubscribe();
                subscriberRef.current = null;
            }
            console.log(`Stopped recording ${selectedTopic}`);
        } else {
            // Start recording
            startTimeRef.current = Date.now();
            console.log(`Starting to record ${selectedTopic}`);
        }
        setIsRecording(!isRecording);
    };

    // Format time for tooltip
    const formatTime = (seconds) => {
        return `${seconds.toFixed(2)}s`;
    };

    // Get current topic configuration
    const getCurrentTopicConfig = () => {
        return plottableTopics.find(t => t.name === selectedTopic);
    };

    return (
        <div className="time-plot-container">
            <div className="time-plot-header">
                <h3>{title}</h3>
                <div className="time-plot-controls">
                    <select
                        value={selectedTopic}
                        onChange={(e) => setSelectedTopic(e.target.value)}
                        disabled={!connected}
                        className="topic-select"
                    >
                        <option value="">Select topic to plot</option>
                        {plottableTopics.map(topic => (
                            <option key={topic.name} value={topic.name}>
                                {topic.name} ({topic.type})
                            </option>
                        ))}
                    </select>

                    <button
                        onClick={toggleRecording}
                        disabled={!connected || !selectedTopic}
                        className={`record-button ${isRecording ? 'recording' : ''}`}
                    >
                        {isRecording ? '⏹ Stop' : '▶ Start'}
                    </button>

                    <button
                        onClick={clearData}
                        className="clear-button"
                    >
                        🗑 Clear
                    </button>
                </div>
            </div>

            <div className="time-plot-content">
                {data.length === 0 ? (
                    <div className="no-data-message">
                        <p>No data to display</p>
                        <p className="sub-message">
                            {!connected ? 'Connect to ROS first' :
                                !selectedTopic ? 'Select a topic to start plotting' :
                                    'Click Start to begin recording data'}
                        </p>
                    </div>
                ) : (
                    <div className="chart-container">
                        <div className="plot-info">
                            <span>Topic: {selectedTopic}</span>
                            <span>Type: {messageType}</span>
                            <span>Data points: {data.length}</span>
                            <span>Window: {WINDOW_SIZE}s</span>
                            <span>Status: {isRecording ? '🔴 Recording' : '⏸ Paused'}</span>
                        </div>

                        <div className="chart-wrapper">
                            <ResponsiveContainer width="100%" height={300}>
                                <LineChart
                                    data={data}
                                    margin={{
                                        top: 20,
                                        right: 5,
                                        left: 0,
                                        bottom: 20,
                                    }}
                                >
                                    <CartesianGrid strokeDasharray="3 3" stroke="#f0f0f0" />
                                    <XAxis
                                        dataKey="time"
                                        type="number"
                                        scale="linear"
                                        domain={['dataMin', 'dataMax']}
                                        tick={false}
                                    />
                                    <YAxis />
                                    <Tooltip
                                        formatter={(value, name) => [value.toFixed(4), 'Value']}
                                        labelFormatter={(time) => `Time: ${formatTime(time)}`}
                                    />
                                    <Legend />
                                    <Line
                                        type="monotone"
                                        dataKey="value"
                                        stroke={getCurrentTopicConfig()?.color || '#8884d8'}
                                        strokeWidth={2}
                                        dot={false}
                                        name={selectedTopic}
                                        connectNulls={false}
                                    />
                                </LineChart>
                            </ResponsiveContainer>
                        </div>

                        <div className="plot-note">
                            <small>📊 Real-time ROS data visualization with {WINDOW_SIZE}s rolling window</small>
                        </div>
                    </div>
                )}
            </div>
        </div>
    );
};

export default TimePlotGraph; 