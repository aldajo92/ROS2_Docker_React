import React, { useState } from 'react';
import ROSLIB from 'roslib';
import useROS from '../hooks/useROS';
import TopicSubscriber from './TopicSubscriber';
import LiveRobotViewer from './LiveRobotViewer';

const ROS2Dashboard = () => {
    const { ros, connected, error } = useROS('ws://localhost:9090');
    const [subscriptions, setSubscriptions] = useState([]);
    const [newTopic, setNewTopic] = useState({ name: '', type: '' });
    const [availableTopics, setAvailableTopics] = useState([]);
    const [showTopics, setShowTopics] = useState(false);
    const [loadingTopics, setLoadingTopics] = useState(false);
    const [showRobot, setShowRobot] = useState(true);

    // Common ROS2 topics for quick access
    const commonTopics = [
        { name: '/clock', type: 'rosgraph_msgs/msg/Clock' },
        { name: '/rosout', type: 'rcl_interfaces/msg/Log' },
        { name: '/parameter_events', type: 'rcl_interfaces/msg/ParameterEvent' },
        { name: '/cmd_vel', type: 'geometry_msgs/msg/Twist' },
        { name: '/odom', type: 'nav_msgs/msg/Odometry' },
        { name: '/scan', type: 'sensor_msgs/msg/LaserScan' },
        { name: '/tf', type: 'tf2_msgs/msg/TFMessage' },
    ];

    const fetchAvailableTopics = () => {
        if (!ros || !connected) {
            console.log('ROS not connected');
            return;
        }

        setLoadingTopics(true);

        const getTopicsClient = new ROSLIB.Service({
            ros: ros,
            name: '/rosapi/topics',
            serviceType: 'rosapi/Topics'
        });

        const request = new ROSLIB.ServiceRequest({});

        getTopicsClient.callService(request, (result) => {
            console.log('Available topics:', result.topics);
            setAvailableTopics(result.topics || []);
            setShowTopics(true);
            setLoadingTopics(false);
        }, (error) => {
            console.error('Failed to get topics:', error);
            // Fallback: try using getTopics method directly
            if (ros && ros.getTopics) {
                ros.getTopics((topics) => {
                    console.log('Available topics (fallback):', topics);
                    setAvailableTopics(topics || []);
                    setShowTopics(true);
                    setLoadingTopics(false);
                }, (error) => {
                    console.error('Failed to get topics with fallback:', error);
                    setLoadingTopics(false);
                });
            } else {
                setLoadingTopics(false);
            }
        });
    };

    const addSubscription = (topicName, messageType) => {
        const exists = subscriptions.find(sub => sub.name === topicName);
        if (!exists && topicName && messageType) {
            setSubscriptions(prev => [...prev, { name: topicName, type: messageType }]);
        }
    };

    const removeSubscription = (topicName) => {
        setSubscriptions(prev => prev.filter(sub => sub.name !== topicName));
    };

    const handleAddCustomTopic = () => {
        if (newTopic.name && newTopic.type) {
            addSubscription(newTopic.name, newTopic.type);
            setNewTopic({ name: '', type: '' });
        }
    };

    return (
        <div className="dashboard">
            <h1>🤖 ROS2 Web Dashboard</h1>

            {/* Connection Status */}
            <div className="status-section">
                <h2>📡 Connection Status</h2>
                <div className="status-row">
                    <div className={`status-indicator ${connected ? 'connected' : 'disconnected'}`}>
                        {connected ? '🟢 Connected' : '🔴 Disconnected'}
                    </div>
                    <div className="connection-info">
                        <p>ROS Bridge: ws://localhost:9090</p>
                        <p>Status: {connected ? 'Active' : 'Waiting for connection...'}</p>
                    </div>
                </div>
            </div>

            {/* Robot Visualization */}
            <div className="robot-section">
                <h2>🎯 Live Robot Visualization</h2>
                <div className="robot-controls">
                    <label>
                        <input
                            type="checkbox"
                            checked={showRobot}
                            onChange={(e) => setShowRobot(e.target.checked)}
                        />
                        Show Robot Model
                    </label>
                </div>

                {showRobot && (
                    <div className="robot-viewer">
                        <LiveRobotViewer
                            ros={ros}
                            connected={connected}
                            height="500px"
                            showGrid={true}
                            showEnvironment={true}
                        />
                    </div>
                )}
            </div>

            {/* Available Topics Section */}
            <div style={{
                padding: '16px',
                marginBottom: '20px',
                borderRadius: '8px',
                backgroundColor: '#f8f9fa',
                border: '1px solid #dee2e6'
            }}>
                <h3>Available Topics</h3>
                <div style={{ marginBottom: '16px' }}>
                    <button
                        onClick={fetchAvailableTopics}
                        disabled={!connected || loadingTopics}
                        style={{
                            padding: '10px 20px',
                            borderRadius: '4px',
                            border: '1px solid #17a2b8',
                            backgroundColor: '#17a2b8',
                            color: 'white',
                            cursor: !connected || loadingTopics ? 'not-allowed' : 'pointer',
                            marginRight: '10px'
                        }}
                    >
                        {loadingTopics ? 'Loading...' : 'List All Topics'}
                    </button>
                    {showTopics && (
                        <button
                            onClick={() => setShowTopics(false)}
                            style={{
                                padding: '10px 20px',
                                borderRadius: '4px',
                                border: '1px solid #6c757d',
                                backgroundColor: '#6c757d',
                                color: 'white',
                                cursor: 'pointer'
                            }}
                        >
                            Hide Topics
                        </button>
                    )}
                </div>

                {showTopics && (
                    <div style={{
                        maxHeight: '300px',
                        overflowY: 'auto',
                        backgroundColor: 'white',
                        border: '1px solid #ddd',
                        borderRadius: '4px',
                        padding: '12px'
                    }}>
                        <h4>Found {availableTopics.length} topics:</h4>
                        {availableTopics.length === 0 ? (
                            <p style={{ color: '#6c757d', fontStyle: 'italic' }}>
                                No topics found. Make sure ROS2 nodes are running.
                            </p>
                        ) : (
                            <ul style={{ margin: 0, paddingLeft: '20px' }}>
                                {availableTopics.map((topic, index) => (
                                    <li key={index} style={{
                                        marginBottom: '4px',
                                        fontFamily: 'monospace',
                                        fontSize: '14px'
                                    }}>
                                        {topic}
                                    </li>
                                ))}
                            </ul>
                        )}
                    </div>
                )}
            </div>

            {/* Quick Topic Selection */}
            <div style={{
                padding: '16px',
                marginBottom: '20px',
                borderRadius: '8px',
                backgroundColor: '#e9ecef',
                border: '1px solid #ced4da'
            }}>
                <h3>Quick Subscribe to Common Topics</h3>
                <div style={{ display: 'flex', flexWrap: 'wrap', gap: '8px' }}>
                    {commonTopics.map(topic => (
                        <button
                            key={topic.name}
                            onClick={() => addSubscription(topic.name, topic.type)}
                            disabled={!connected || subscriptions.some(sub => sub.name === topic.name)}
                            style={{
                                padding: '8px 12px',
                                borderRadius: '4px',
                                border: '1px solid #007bff',
                                backgroundColor: subscriptions.some(sub => sub.name === topic.name) ? '#6c757d' : '#007bff',
                                color: 'white',
                                cursor: !connected || subscriptions.some(sub => sub.name === topic.name) ? 'not-allowed' : 'pointer'
                            }}
                        >
                            {topic.name}
                        </button>
                    ))}
                </div>
            </div>

            {/* Custom Topic Subscription */}
            <div style={{
                padding: '16px',
                marginBottom: '20px',
                borderRadius: '8px',
                backgroundColor: '#fff3cd',
                border: '1px solid #ffeaa7'
            }}>
                <h3>Subscribe to Custom Topic</h3>
                <div style={{ display: 'flex', gap: '8px', alignItems: 'center', flexWrap: 'wrap' }}>
                    <input
                        type="text"
                        placeholder="Topic name (e.g., /my_topic)"
                        value={newTopic.name}
                        onChange={(e) => setNewTopic(prev => ({ ...prev, name: e.target.value }))}
                        style={{ padding: '8px', borderRadius: '4px', border: '1px solid #ccc', minWidth: '200px' }}
                    />
                    <input
                        type="text"
                        placeholder="Message type (e.g., std_msgs/msg/String)"
                        value={newTopic.type}
                        onChange={(e) => setNewTopic(prev => ({ ...prev, type: e.target.value }))}
                        style={{ padding: '8px', borderRadius: '4px', border: '1px solid #ccc', minWidth: '250px' }}
                    />
                    <button
                        onClick={handleAddCustomTopic}
                        disabled={!connected || !newTopic.name || !newTopic.type}
                        style={{
                            padding: '8px 16px',
                            borderRadius: '4px',
                            border: '1px solid #28a745',
                            backgroundColor: '#28a745',
                            color: 'white',
                            cursor: !connected || !newTopic.name || !newTopic.type ? 'not-allowed' : 'pointer'
                        }}
                    >
                        Subscribe
                    </button>
                </div>
            </div>

            {/* Active Subscriptions */}
            <div>
                <h3>Active Subscriptions ({subscriptions.length})</h3>
                {subscriptions.length === 0 ? (
                    <p style={{ color: '#6c757d', fontStyle: 'italic' }}>
                        No active subscriptions. Add some topics above to start receiving data.
                    </p>
                ) : (
                    subscriptions.map(sub => (
                        <div key={sub.name} style={{ position: 'relative' }}>
                            <button
                                onClick={() => removeSubscription(sub.name)}
                                style={{
                                    position: 'absolute',
                                    top: '16px',
                                    right: '16px',
                                    background: '#dc3545',
                                    color: 'white',
                                    border: 'none',
                                    borderRadius: '4px',
                                    padding: '4px 8px',
                                    cursor: 'pointer',
                                    zIndex: 1
                                }}
                            >
                                ✕ Remove
                            </button>
                            <TopicSubscriber
                                ros={ros}
                                topicName={sub.name}
                                messageType={sub.type}
                            />
                        </div>
                    ))
                )}
            </div>

            {/* Inline CSS */}
            <style jsx>{`
                .dashboard {
                    padding: 20px;
                    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                    background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);
                    min-height: 100vh;
                }

                .dashboard h1 {
                    text-align: center;
                    color: #2c3e50;
                    margin-bottom: 30px;
                    font-size: 2.5em;
                    text-shadow: 2px 2px 4px rgba(0,0,0,0.1);
                }

                .status-section, .robot-section, .topics-section {
                    background: white;
                    border-radius: 12px;
                    padding: 20px;
                    margin-bottom: 25px;
                    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
                    border: 1px solid #e1e8ed;
                }

                .status-section h2, .robot-section h2, .topics-section h2 {
                    color: #34495e;
                    margin-bottom: 15px;
                    font-size: 1.4em;
                    border-bottom: 2px solid #3498db;
                    padding-bottom: 8px;
                }

                .status-row {
                    display: flex;
                    align-items: center;
                    gap: 15px;
                }

                .status-indicator {
                    padding: 12px 20px;
                    border-radius: 25px;
                    font-weight: bold;
                    font-size: 1.1em;
                    min-width: 150px;
                    text-align: center;
                }

                .status-indicator.connected {
                    background: linear-gradient(135deg, #2ecc71, #27ae60);
                    color: white;
                    box-shadow: 0 2px 4px rgba(46, 204, 113, 0.3);
                }

                .status-indicator.disconnected {
                    background: linear-gradient(135deg, #e74c3c, #c0392b);
                    color: white;
                    box-shadow: 0 2px 4px rgba(231, 76, 60, 0.3);
                }

                .connection-info p {
                    margin: 5px 0;
                    color: #5a6c7d;
                    font-size: 0.95em;
                }

                .robot-controls {
                    margin-bottom: 15px;
                }

                .robot-controls label {
                    display: flex;
                    align-items: center;
                    gap: 8px;
                    font-size: 1.1em;
                    color: #2c3e50;
                    cursor: pointer;
                }

                .robot-controls input[type="checkbox"] {
                    width: 18px;
                    height: 18px;
                    accent-color: #3498db;
                }

                .robot-viewer {
                    border-radius: 8px;
                    overflow: hidden;
                    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
                }

                .topics-controls {
                    display: flex;
                    gap: 10px;
                    margin-bottom: 15px;
                    flex-wrap: wrap;
                }

                .topics-controls button {
                    padding: 10px 20px;
                    border: none;
                    border-radius: 6px;
                    background: linear-gradient(135deg, #3498db, #2980b9);
                    color: white;
                    cursor: pointer;
                    font-weight: 500;
                    transition: all 0.3s ease;
                    box-shadow: 0 2px 4px rgba(52, 152, 219, 0.3);
                }

                .topics-controls button:hover {
                    transform: translateY(-2px);
                    box-shadow: 0 4px 8px rgba(52, 152, 219, 0.4);
                }

                .topics-controls button:disabled {
                    background: linear-gradient(135deg, #95a5a6, #7f8c8d);
                    cursor: not-allowed;
                    transform: none;
                }

                .loading-spinner {
                    text-align: center;
                    color: #7f8c8d;
                    font-style: italic;
                }

                .topic-list {
                    background: #f8f9fa;
                    border-radius: 8px;
                    padding: 15px;
                    max-height: 300px;
                    overflow-y: auto;
                }

                .topic-item {
                    background: white;
                    border: 1px solid #dee2e6;
                    border-radius: 6px;
                    padding: 12px;
                    margin-bottom: 8px;
                    transition: all 0.2s ease;
                }

                .topic-item:hover {
                    border-color: #3498db;
                    box-shadow: 0 2px 4px rgba(52, 152, 219, 0.2);
                }

                .topic-name {
                    font-weight: 600;
                    color: #2c3e50;
                    margin-bottom: 4px;
                }

                .topic-type {
                    font-size: 0.9em;
                    color: #7f8c8d;
                    font-family: monospace;
                }

                @media (max-width: 768px) {
                    .dashboard {
                        padding: 15px;
                    }
                    
                    .dashboard h1 {
                        font-size: 2em;
                    }
                    
                    .status-row {
                        flex-direction: column;
                        align-items: flex-start;
                        gap: 10px;
                    }
                    
                    .topics-controls {
                        flex-direction: column;
                    }
                }
            `}</style>
        </div>
    );
};

export default ROS2Dashboard; 