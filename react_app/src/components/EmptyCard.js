import React from 'react';
import './EmptyCard.css';

const EmptyCard = () => {
    return (
        <div className="empty-card">
            <div className="empty-card-content">
                <h3 className="empty-card-title">Empty Panel</h3>
                <p className="empty-card-description">
                    This space can be used for additional components or visualizations
                </p>
            </div>
        </div>
    );
};

export default EmptyCard; 