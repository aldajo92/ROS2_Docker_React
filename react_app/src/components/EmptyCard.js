import React from 'react';

const EmptyCard = () => {
    return (
        <div style={{
            padding: '20px',
            borderRadius: '8px',
            backgroundColor: '#f8f9fa',
            border: '2px dashed #dee2e6',
            height: '100%',
            width: '100%',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            minHeight: '600px'
        }}>
            <div style={{
                textAlign: 'center',
                color: '#6c757d'
            }}>
                <h3 style={{ marginBottom: '10px' }}>Empty Panel</h3>
                <p style={{ fontSize: '14px', fontStyle: 'italic' }}>
                    This space can be used for additional components or visualizations
                </p>
            </div>
        </div>
    );
};

export default EmptyCard; 