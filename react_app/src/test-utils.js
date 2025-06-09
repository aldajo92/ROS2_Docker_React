import React from 'react';
import { render } from '@testing-library/react';
import ROSLIB from 'roslib';

// Mock ROSLIB globally
export const createMockRos = (overrides = {}) => ({
    on: jest.fn(),
    close: jest.fn(),
    getTopics: jest.fn(),
    ...overrides
});

export const createMockTopic = (overrides = {}) => ({
    subscribe: jest.fn(),
    unsubscribe: jest.fn(),
    ...overrides
});

export const createMockService = (overrides = {}) => ({
    callService: jest.fn(),
    ...overrides
});

// Custom render function with providers
export const renderWithProviders = (ui, options = {}) => {
    return render(ui, {
        // Add any global providers here if needed in the future
        ...options,
    });
};

// Mock message generators for testing
export const createMockMessage = (type = 'string', data = 'test message') => {
    switch (type) {
        case 'string':
            return { data };
        case 'twist':
            return {
                linear: { x: 1.0, y: 0.0, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: 0.5 }
            };
        case 'odometry':
            return {
                header: {
                    stamp: { sec: 123, nanosec: 456789 },
                    frame_id: 'odom'
                },
                pose: {
                    pose: {
                        position: { x: 1.0, y: 2.0, z: 0.0 },
                        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
                    }
                },
                twist: {
                    twist: {
                        linear: { x: 0.5, y: 0.0, z: 0.0 },
                        angular: { x: 0.0, y: 0.0, z: 0.1 }
                    }
                }
            };
        case 'laser_scan':
            return {
                header: {
                    stamp: { sec: 123, nanosec: 456789 },
                    frame_id: 'laser'
                },
                angle_min: -1.57,
                angle_max: 1.57,
                angle_increment: 0.01,
                time_increment: 0.0,
                scan_time: 0.1,
                range_min: 0.1,
                range_max: 10.0,
                ranges: [1.0, 2.0, 3.0, 4.0, 5.0],
                intensities: []
            };
        default:
            return { data };
    }
};

// Common topic names for testing
export const COMMON_TOPICS = {
    CLOCK: '/clock',
    ROSOUT: '/rosout',
    CMD_VEL: '/cmd_vel',
    ODOM: '/odom',
    SCAN: '/scan',
    TF: '/tf'
};

// Common message types for testing
export const MESSAGE_TYPES = {
    STRING: 'std_msgs/msg/String',
    TWIST: 'geometry_msgs/msg/Twist',
    ODOMETRY: 'nav_msgs/msg/Odometry',
    LASER_SCAN: 'sensor_msgs/msg/LaserScan',
    CLOCK: 'rosgraph_msgs/msg/Clock',
    LOG: 'rcl_interfaces/msg/Log'
};

// Wait for async operations in tests
export const waitForAsync = () => new Promise(resolve => setTimeout(resolve, 0));

export * from '@testing-library/react'; 