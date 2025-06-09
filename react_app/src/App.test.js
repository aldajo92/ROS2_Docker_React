import React from 'react';
import { render, screen } from '@testing-library/react';
import App from './App';

// Mock the ROS2Dashboard component
jest.mock('./components/ROS2Dashboard', () => {
  return function MockROS2Dashboard() {
    return <div data-testid="ros2-dashboard">ROS2 Dashboard Mock</div>;
  };
});

describe('App Component', () => {
  it('should render without crashing', () => {
    render(<App />);
  });

  it('should render ROS2Dashboard component', () => {
    render(<App />);
    expect(screen.getByTestId('ros2-dashboard')).toBeInTheDocument();
  });

  it('should have correct CSS class', () => {
    const { container } = render(<App />);
    expect(container.firstChild).toHaveClass('App');
  });
});
