import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';

const TopicSubscriber = ({ ros, topicName, messageType }) => {
    const [messages, setMessages] = useState([]);
    const [isSubscribed, setIsSubscribed] = useState(false);
    const [lastMessage, setLastMessage] = useState(null);

    useEffect(() => {
        if (!ros || !topicName || !messageType) return;

        const listener = new ROSLIB.Topic({
            ros: ros,
            name: topicName,
            messageType: messageType
        });

        listener.subscribe((message) => {
            console.log(`Received message on ${topicName}:`, message);
            setLastMessage(message);
            setMessages(prev => [
                { timestamp: new Date().toISOString(), data: message },
                ...prev.slice(0, 9) // Keep only last 10 messages
            ]);
        });

        setIsSubscribed(true);

        return () => {
            listener.unsubscribe();
            setIsSubscribed(false);
        };
    }, [ros, topicName, messageType]);

    const formatMessage = (msg) => {
        if (typeof msg === 'object') {
            return JSON.stringify(msg, null, 2);
        }
        return msg.toString();
    };

    return (
        <div style={{
            border: '1px solid #ccc',
            borderRadius: '8px',
            padding: '16px',
            margin: '16px 0',
            backgroundColor: '#f9f9f9'
        }}>
            <h3>Topic: {topicName}</h3>
            <p>Type: {messageType}</p>
            <p>Status: {isSubscribed ? '🟢 Subscribed' : '🔴 Not Subscribed'}</p>

            {lastMessage && (
                <div>
                    <h4>Latest Message:</h4>
                    <pre style={{
                        backgroundColor: '#fff',
                        padding: '8px',
                        borderRadius: '4px',
                        fontSize: '12px',
                        overflow: 'auto',
                        maxHeight: '200px'
                    }}>
                        {formatMessage(lastMessage)}
                    </pre>
                </div>
            )}

            {messages.length > 0 && (
                <div>
                    <h4>Message History:</h4>
                    <div style={{ maxHeight: '300px', overflowY: 'auto' }}>
                        {messages.map((msg, index) => (
                            <div key={index} style={{
                                marginBottom: '8px',
                                fontSize: '12px',
                                borderBottom: '1px solid #eee',
                                paddingBottom: '4px'
                            }}>
                                <strong>{msg.timestamp}</strong>
                                <pre style={{ margin: '4px 0', fontSize: '11px' }}>
                                    {formatMessage(msg.data)}
                                </pre>
                            </div>
                        ))}
                    </div>
                </div>
            )}
        </div>
    );
};

export default TopicSubscriber; 