import React, { useState } from 'react';
import ROSLIB from 'roslib';
import useROS from '../hooks/useROS';
import ConnectionStatus from './ConnectionStatus';
import AvailableTopics from './AvailableTopics';
import EmptyCard from './EmptyCard';
import './ROS2Dashboard.css';

const ROS2Dashboard = () => {
    const { ros, connected, error } = useROS('ws://localhost:9090');
    const [subscriptions, setSubscriptions] = useState([]);

    const addSubscription = (topicName, messageType) => {
        const exists = subscriptions.find(sub => sub.name === topicName);
        if (!exists && topicName && messageType) {
            setSubscriptions(prev => [...prev, { name: topicName, type: messageType }]);
        }
    };

    const removeSubscription = (topicName) => {
        setSubscriptions(prev => prev.filter(sub => sub.name !== topicName));
    };

    return (
        <div className="dashboard-container">
            <h1 className="dashboard-header">ROS2 Web Dashboard</h1>

            <div className="dashboard-content">
                {/* Left Panel */}
                <div className="left-panel">
                    {/* Connection Status */}
                    <ConnectionStatus
                        connected={connected}
                        error={error}
                        url="ws://localhost:9090"
                    />

                    {/* Available Topics Section */}
                    <AvailableTopics
                        ros={ros}
                        connected={connected}
                    />
                </div>

                {/* Right Panel */}
                <div className="right-panel">
                    <EmptyCard />
                </div>
            </div>
        </div>
    );
};

export default ROS2Dashboard; 