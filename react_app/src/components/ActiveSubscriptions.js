import React from 'react';
import TopicSubscriber from './TopicSubscriber';

const ActiveSubscriptions = ({ subscriptions, ros, onRemoveSubscription }) => {
    return (
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
                            onClick={() => onRemoveSubscription(sub.name)}
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
    );
};

export default ActiveSubscriptions; 