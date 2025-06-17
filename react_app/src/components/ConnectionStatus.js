import React from 'react';

const ConnectionStatus = ({ connected, error, url = 'ws://localhost:9090' }) => {
    return (
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
            <p>URL: {url}</p>
            {error && <p>Error: {error}</p>}
        </div>
    );
};

export default ConnectionStatus; 