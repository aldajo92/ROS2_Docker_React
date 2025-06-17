import React, { useState } from 'react';
import ROSLIB from 'roslib';
import useROS from '../hooks/useROS';
import ConnectionStatus from './ConnectionStatus';
import AvailableTopics from './AvailableTopics';
import EmptyCard from './EmptyCard';

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
        <div style={{
            padding: '20px',
            fontFamily: 'Arial, sans-serif',
            height: '100vh',
            display: 'flex',
            flexDirection: 'column'
        }}>
            <h1 style={{ marginBottom: '20px' }}>ROS2 Web Dashboard</h1>

            <div style={{
                display: 'flex',
                gap: '20px',
                flex: 1,
                height: 'calc(100vh - 100px)' // Adjust based on header height
            }}>
                {/* Left Panel */}
                <div style={{
                    minWidth: '400px',
                    maxWidth: '500px',
                    display: 'flex',
                    flexDirection: 'column',
                    gap: '0'
                }}>
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
                <div style={{
                    flex: 1,
                    display: 'flex'
                }}>
                    <EmptyCard />
                </div>
            </div>
        </div>
    );
};

export default ROS2Dashboard; 