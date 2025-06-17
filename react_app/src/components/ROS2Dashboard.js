import React, { useState } from 'react';
import ROSLIB from 'roslib';
import useROS from '../hooks/useROS';
import ConnectionStatus from './ConnectionStatus';
import AvailableTopics from './AvailableTopics';
import QuickTopicSelection from './QuickTopicSelection';
import CustomTopicSubscription from './CustomTopicSubscription';
import ActiveSubscriptions from './ActiveSubscriptions';

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
        <div style={{ padding: '20px', fontFamily: 'Arial, sans-serif' }}>
            <h1>ROS2 Web Dashboard</h1>

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

            {/* Quick Topic Selection */}
            <QuickTopicSelection
                connected={connected}
                subscriptions={subscriptions}
                onAddSubscription={addSubscription}
            />

            {/* Custom Topic Subscription */}
            <CustomTopicSubscription
                connected={connected}
                onAddSubscription={addSubscription}
            />

            {/* Active Subscriptions */}
            <ActiveSubscriptions
                subscriptions={subscriptions}
                ros={ros}
                onRemoveSubscription={removeSubscription}
            />
        </div>
    );
};

export default ROS2Dashboard; 