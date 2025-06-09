import { renderHook, act } from '@testing-library/react';
import useROS from '../useROS';
import ROSLIB from 'roslib';

// Mock ROSLIB
jest.mock('roslib');

describe('useROS Hook', () => {
    let mockRos;
    let mockEventListeners;

    beforeEach(() => {
        mockEventListeners = {};
        mockRos = {
            on: jest.fn((event, callback) => {
                mockEventListeners[event] = callback;
            }),
            close: jest.fn()
        };

        ROSLIB.Ros.mockImplementation(() => mockRos);
    });

    afterEach(() => {
        jest.clearAllMocks();
    });

    it('should initialize ROS connection with default URL', () => {
        renderHook(() => useROS());

        expect(ROSLIB.Ros).toHaveBeenCalledWith({
            url: 'ws://localhost:9090'
        });
    });

    it('should initialize ROS connection with custom URL', () => {
        const customUrl = 'ws://localhost:8080';
        renderHook(() => useROS(customUrl));

        expect(ROSLIB.Ros).toHaveBeenCalledWith({
            url: customUrl
        });
    });

    it('should set up event listeners on connection', () => {
        renderHook(() => useROS());

        expect(mockRos.on).toHaveBeenCalledWith('connection', expect.any(Function));
        expect(mockRos.on).toHaveBeenCalledWith('error', expect.any(Function));
        expect(mockRos.on).toHaveBeenCalledWith('close', expect.any(Function));
    });

    it('should handle successful connection', () => {
        const { result } = renderHook(() => useROS());

        expect(result.current.connected).toBe(false);
        expect(result.current.error).toBe(null);

        // Simulate successful connection
        act(() => {
            mockEventListeners.connection();
        });

        expect(result.current.connected).toBe(true);
        expect(result.current.error).toBe(null);
    });

    it('should handle connection error', () => {
        const { result } = renderHook(() => useROS());
        const errorMessage = 'Connection failed';

        expect(result.current.connected).toBe(false);
        expect(result.current.error).toBe(null);

        // Simulate connection error
        act(() => {
            mockEventListeners.error(new Error(errorMessage));
        });

        expect(result.current.connected).toBe(false);
        expect(result.current.error).toBe(`Error: ${errorMessage}`);
    });

    it('should handle connection close', () => {
        const { result } = renderHook(() => useROS());

        // First connect
        act(() => {
            mockEventListeners.connection();
        });
        expect(result.current.connected).toBe(true);

        // Then close connection
        act(() => {
            mockEventListeners.close();
        });

        expect(result.current.connected).toBe(false);
    });

    it('should clean up on unmount', () => {
        const { unmount } = renderHook(() => useROS());

        unmount();

        expect(mockRos.close).toHaveBeenCalled();
    });

    it('should handle URL change by creating new connection', () => {
        let url = 'ws://localhost:9090';
        const { rerender } = renderHook(({ url }) => useROS(url), {
            initialProps: { url }
        });

        expect(ROSLIB.Ros).toHaveBeenCalledTimes(1);
        expect(ROSLIB.Ros).toHaveBeenLastCalledWith({ url: 'ws://localhost:9090' });

        // Change URL
        url = 'ws://localhost:8080';
        rerender({ url });

        expect(ROSLIB.Ros).toHaveBeenCalledTimes(2);
        expect(ROSLIB.Ros).toHaveBeenLastCalledWith({ url: 'ws://localhost:8080' });
        expect(mockRos.close).toHaveBeenCalled();
    });
}); 