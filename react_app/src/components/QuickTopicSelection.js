import React from 'react';

const QuickTopicSelection = ({ connected, subscriptions, onAddSubscription }) => {
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

    return (
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
                        onClick={() => onAddSubscription(topic.name, topic.type)}
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
    );
};

export default QuickTopicSelection; 