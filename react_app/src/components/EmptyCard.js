import React, { useState } from 'react';
import './EmptyCard.css';

const EmptyCard = () => {
    const [imageError, setImageError] = useState(false);
    const [isVerticallyFlipped, setIsVerticallyFlipped] = useState(false);
    const [isHorizontallyFlipped, setIsHorizontallyFlipped] = useState(false);
    const streamUrl = 'http://waverbot01.local:8080/stream?topic=/camera/image_raw';

    const handleImageError = () => {
        setImageError(true);
    };

    const handleImageLoad = () => {
        setImageError(false);
    };

    return (
        <div className="empty-card">
            {imageError ? (
                <div className="empty-card-content">
                    <h3 className="empty-card-title">Camera Stream</h3>
                    <p className="empty-card-description">
                        Unable to connect to camera stream
                    </p>
                    <p className="stream-url">
                        <small>{streamUrl}</small>
                    </p>
                    <button
                        onClick={() => setImageError(false)}
                        className="retry-button"
                    >
                        Retry Connection
                    </button>
                </div>
            ) : (
                <div className="camera-stream-container">
                    <h3 className="stream-title">Camera Stream</h3>
                    <div className="stream-wrapper">
                        <div className="flip-controls">
                            <button
                                onClick={() => setIsVerticallyFlipped(!isVerticallyFlipped)}
                                className="flip-button vertical-flip"
                                title={isVerticallyFlipped ? "Restore vertical flip" : "Flip image vertically"}
                            >
                                ↕️
                            </button>
                            <button
                                onClick={() => setIsHorizontallyFlipped(!isHorizontallyFlipped)}
                                className="flip-button horizontal-flip"
                                title={isHorizontallyFlipped ? "Restore horizontal flip" : "Flip image horizontally"}
                            >
                                ↔️
                            </button>
                        </div>
                        <img
                            src={streamUrl}
                            alt="Camera Stream"
                            className={`camera-stream ${isVerticallyFlipped ? 'flipped-vertical' : ''} ${isHorizontallyFlipped ? 'flipped-horizontal' : ''}`}
                            onError={handleImageError}
                            onLoad={handleImageLoad}
                        />
                    </div>
                    <p className="stream-info">
                        <small>Streaming from: waverbot01.local:8080</small>
                    </p>
                </div>
            )}
        </div>
    );
};

export default EmptyCard; 