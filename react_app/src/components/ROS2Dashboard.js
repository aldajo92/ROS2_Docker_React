import React, { useState } from 'react';
import ROSLIB from 'roslib';
import useROS from '../hooks/useROS';
import TopicSubscriber from './TopicSubscriber';

const ROS2Dashboard = () => {
    const { ros, connected, error } = useROS('ws://localhost:9090');
    const [subscriptions, setSubscriptions] = useState([]);
    const [newTopic, setNewTopic] = useState({ name: '', type: '' });
    const [availableTopics, setAvailableTopics] = useState([]);
    const [showTopics, setShowTopics] = useState(false);
    const [loadingTopics, setLoadingTopics] = useState(false);

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
        <div style={{ padding: '20px', fontFamily: 'Arial, sans-serif' }}>
            <h1>ROS2 Web Dashboard</h1>

            {/* Connection Status */}
            <div style={{
                padding: '16px',
                marginBottom: '20px',
                borderRadius: '8px',
                backgroundColor: connected ? '#d4edda' : '#f8d7da',
                border: `1px solid ${connected ? '#c3e6cb' : '#f5c6cb'}`,
                color: connected ? '#155724' : '#721c24'
            }}>
                <h3>Connection Status</h3>
                <p>ROS Bridge: {connected ? '🟢 Connected' : '🔴 Disconnected'}</p>
                <p>URL: ws://localhost:9090</p>
                {error && <p>Error: {error}</p>}
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
        </div>
    );
};

export default ROS2Dashboard; 