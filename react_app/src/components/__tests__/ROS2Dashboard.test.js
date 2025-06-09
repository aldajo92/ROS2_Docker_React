import React from 'react';
import { render, screen, fireEvent, waitFor, act } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import ROS2Dashboard from '../ROS2Dashboard';
import useROS from '../../hooks/useROS';
import ROSLIB from 'roslib';

// Mock the hooks and ROSLIB
jest.mock('../../hooks/useROS');
jest.mock('roslib');

// Mock TopicSubscriber component
jest.mock('../TopicSubscriber', () => {
    return function MockTopicSubscriber({ topicName, messageType }) {
        return (
            <div data-testid="topic-subscriber">
                <span>Topic: {topicName}</span>
                <span>Type: {messageType}</span>
            </div>
        );
    };
});

describe('ROS2Dashboard Component', () => {
    let mockRos;
    let mockService;
    let mockServiceCallback;

    beforeEach(() => {
        mockServiceCallback = null;
        mockService = {
            callService: jest.fn((request, successCallback, errorCallback) => {
                mockServiceCallback = { successCallback, errorCallback };
            })
        };

        mockRos = {
            getTopics: jest.fn()
        };

        ROSLIB.Service.mockImplementation(() => mockService);
        ROSLIB.ServiceRequest.mockImplementation(() => ({}));

        // Default mock implementation for useROS
        useROS.mockReturnValue({
            ros: mockRos,
            connected: false,
            error: null
        });
    });

    afterEach(() => {
        jest.clearAllMocks();
    });

    it('should render dashboard title', () => {
        render(<ROS2Dashboard />);
        expect(screen.getByText('ROS2 Web Dashboard')).toBeInTheDocument();
    });

    it('should show disconnected status when not connected', () => {
        render(<ROS2Dashboard />);

        expect(screen.getByText('ROS Bridge: 🔴 Disconnected')).toBeInTheDocument();
        expect(screen.getByText('URL: ws://localhost:9090')).toBeInTheDocument();
    });

    it('should show connected status when connected', () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);
        expect(screen.getByText('ROS Bridge: 🟢 Connected')).toBeInTheDocument();
    });

    it('should display error message when there is an error', () => {
        const errorMessage = 'Connection failed';
        useROS.mockReturnValue({
            ros: mockRos,
            connected: false,
            error: errorMessage
        });

        render(<ROS2Dashboard />);
        expect(screen.getByText(`Error: ${errorMessage}`)).toBeInTheDocument();
    });

    it('should render common topic buttons', () => {
        render(<ROS2Dashboard />);

        expect(screen.getByText('/clock')).toBeInTheDocument();
        expect(screen.getByText('/rosout')).toBeInTheDocument();
        expect(screen.getByText('/cmd_vel')).toBeInTheDocument();
        expect(screen.getByText('/odom')).toBeInTheDocument();
    });

    it('should disable topic buttons when not connected', () => {
        render(<ROS2Dashboard />);

        const clockButton = screen.getByText('/clock');
        expect(clockButton).toBeDisabled();
    });

    it('should enable topic buttons when connected', () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const clockButton = screen.getByText('/clock');
        expect(clockButton).not.toBeDisabled();
    });

    it('should add subscription when common topic button is clicked', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const clockButton = screen.getByText('/clock');
        await userEvent.click(clockButton);

        expect(screen.getByTestId('topic-subscriber')).toBeInTheDocument();
        expect(screen.getByText('Topic: /clock')).toBeInTheDocument();
        expect(screen.getByText('Type: rosgraph_msgs/msg/Clock')).toBeInTheDocument();
    });

    it('should handle custom topic subscription', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const topicInput = screen.getByPlaceholderText('Topic name (e.g., /my_topic)');
        const typeInput = screen.getByPlaceholderText('Message type (e.g., std_msgs/msg/String)');
        const subscribeButton = screen.getByText('Subscribe');

        await userEvent.type(topicInput, '/custom_topic');
        await userEvent.type(typeInput, 'std_msgs/msg/String');
        await userEvent.click(subscribeButton);

        expect(screen.getByText('Topic: /custom_topic')).toBeInTheDocument();
        expect(screen.getByText('Type: std_msgs/msg/String')).toBeInTheDocument();
    });

    it('should clear custom topic inputs after subscription', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const topicInput = screen.getByPlaceholderText('Topic name (e.g., /my_topic)');
        const typeInput = screen.getByPlaceholderText('Message type (e.g., std_msgs/msg/String)');
        const subscribeButton = screen.getByText('Subscribe');

        await userEvent.type(topicInput, '/custom_topic');
        await userEvent.type(typeInput, 'std_msgs/msg/String');
        await userEvent.click(subscribeButton);

        expect(topicInput.value).toBe('');
        expect(typeInput.value).toBe('');
    });

    it('should not allow duplicate subscriptions', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const clockButton = screen.getByText('/clock');
        await userEvent.click(clockButton);
        await userEvent.click(clockButton);

        const subscriptions = screen.getAllByTestId('topic-subscriber');
        expect(subscriptions).toHaveLength(1);
    });

    it('should remove subscription when remove button is clicked', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const clockButton = screen.getByText('/clock');
        await userEvent.click(clockButton);

        expect(screen.getByTestId('topic-subscriber')).toBeInTheDocument();

        const removeButton = screen.getByText('✕ Remove');
        await userEvent.click(removeButton);

        expect(screen.queryByTestId('topic-subscriber')).not.toBeInTheDocument();
    });

    it('should display subscription count', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        expect(screen.getByText('Active Subscriptions (0)')).toBeInTheDocument();

        const clockButton = screen.getByText('/clock');
        await userEvent.click(clockButton);

        expect(screen.getByText('Active Subscriptions (1)')).toBeInTheDocument();
    });

    it('should show "List All Topics" button', () => {
        render(<ROS2Dashboard />);
        expect(screen.getByText('List All Topics')).toBeInTheDocument();
    });

    it('should disable "List All Topics" button when not connected', () => {
        render(<ROS2Dashboard />);
        const listTopicsButton = screen.getByText('List All Topics');
        expect(listTopicsButton).toBeDisabled();
    });

    it('should enable "List All Topics" button when connected', () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);
        const listTopicsButton = screen.getByText('List All Topics');
        expect(listTopicsButton).not.toBeDisabled();
    });

    it('should call ROS service when "List All Topics" is clicked', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const listTopicsButton = screen.getByText('List All Topics');
        await userEvent.click(listTopicsButton);

        expect(ROSLIB.Service).toHaveBeenCalledWith({
            ros: mockRos,
            name: '/rosapi/topics',
            serviceType: 'rosapi/Topics'
        });
        expect(mockService.callService).toHaveBeenCalled();
    });

    it('should display topics when service call succeeds', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const listTopicsButton = screen.getByText('List All Topics');
        await userEvent.click(listTopicsButton);

        // Simulate successful service response wrapped in act()
        await act(async () => {
            const mockTopics = ['/topic1', '/topic2', '/topic3'];
            if (mockServiceCallback) {
                mockServiceCallback.successCallback({ topics: mockTopics });
            }
        });

        await waitFor(() => {
            expect(screen.getByText('Found 3 topics:')).toBeInTheDocument();
            expect(screen.getByText('/topic1')).toBeInTheDocument();
            expect(screen.getByText('/topic2')).toBeInTheDocument();
            expect(screen.getByText('/topic3')).toBeInTheDocument();
        });
    });

    it('should show loading state when fetching topics', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const listTopicsButton = screen.getByText('List All Topics');
        await userEvent.click(listTopicsButton);

        expect(screen.getByText('Loading...')).toBeInTheDocument();
    });

    it('should fallback to ros.getTopics when service fails', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const listTopicsButton = screen.getByText('List All Topics');
        await userEvent.click(listTopicsButton);

        // Simulate service error and successful fallback wrapped in act()
        await act(async () => {
            const mockTopics = ['/fallback_topic1', '/fallback_topic2'];
            mockRos.getTopics.mockImplementation((successCallback) => {
                successCallback(mockTopics);
            });

            if (mockServiceCallback) {
                mockServiceCallback.errorCallback(new Error('Service failed'));
            }
        });

        await waitFor(() => {
            expect(mockRos.getTopics).toHaveBeenCalled();
            expect(screen.getByText('Found 2 topics:')).toBeInTheDocument();
        });
    });

    it('should hide topics when "Hide Topics" button is clicked', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const listTopicsButton = screen.getByText('List All Topics');
        await userEvent.click(listTopicsButton);

        // Simulate successful service response wrapped in act()
        await act(async () => {
            if (mockServiceCallback) {
                mockServiceCallback.successCallback({ topics: ['/topic1'] });
            }
        });

        await waitFor(() => {
            expect(screen.getByText('Hide Topics')).toBeInTheDocument();
        });

        const hideTopicsButton = screen.getByText('Hide Topics');
        await userEvent.click(hideTopicsButton);

        expect(screen.queryByText('Found 1 topics:')).not.toBeInTheDocument();
    });

    it('should show no topics message when topics array is empty', async () => {
        useROS.mockReturnValue({
            ros: mockRos,
            connected: true,
            error: null
        });

        render(<ROS2Dashboard />);

        const listTopicsButton = screen.getByText('List All Topics');
        await userEvent.click(listTopicsButton);

        // Simulate service response with empty topics wrapped in act()
        await act(async () => {
            if (mockServiceCallback) {
                mockServiceCallback.successCallback({ topics: [] });
            }
        });

        await waitFor(() => {
            expect(screen.getByText('No topics found. Make sure ROS2 nodes are running.')).toBeInTheDocument();
        });
    });
}); 