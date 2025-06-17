import React, { useState } from 'react';
import ROSLIB from 'roslib';

const AvailableTopics = ({ ros, connected }) => {
    const [availableTopics, setAvailableTopics] = useState([]);
    const [showTopics, setShowTopics] = useState(false);
    const [loadingTopics, setLoadingTopics] = useState(false);

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

    return (
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
    );
};

export default AvailableTopics; 