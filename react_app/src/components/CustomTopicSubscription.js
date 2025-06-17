import React, { useState } from 'react';

const CustomTopicSubscription = ({ connected, onAddSubscription }) => {
    const [newTopic, setNewTopic] = useState({ name: '', type: '' });

    const handleAddCustomTopic = () => {
        if (newTopic.name && newTopic.type) {
            onAddSubscription(newTopic.name, newTopic.type);
            setNewTopic({ name: '', type: '' });
        }
    };

    const handleKeyPress = (e) => {
        if (e.key === 'Enter') {
            handleAddCustomTopic();
        }
    };

    return (
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
                    onKeyPress={handleKeyPress}
                    style={{ padding: '8px', borderRadius: '4px', border: '1px solid #ccc', minWidth: '200px' }}
                />
                <input
                    type="text"
                    placeholder="Message type (e.g., std_msgs/msg/String)"
                    value={newTopic.type}
                    onChange={(e) => setNewTopic(prev => ({ ...prev, type: e.target.value }))}
                    onKeyPress={handleKeyPress}
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
    );
};

export default CustomTopicSubscription; 