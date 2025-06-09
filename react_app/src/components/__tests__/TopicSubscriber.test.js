import React from 'react';
import { render, screen } from '@testing-library/react';
import TopicSubscriber from '../TopicSubscriber';

describe('TopicSubscriber Component', () => {
    it('should show not subscribed status when ros is null', () => {
        render(<TopicSubscriber ros={null} topicName="/test_topic" messageType="std_msgs/msg/String" />);

        expect(screen.getByText('Topic: /test_topic')).toBeInTheDocument();
        expect(screen.getByText('Type: std_msgs/msg/String')).toBeInTheDocument();
        expect(screen.getByText('Status: 🔴 Not Subscribed')).toBeInTheDocument();
    });

    it('should show not subscribed status when topicName is null', () => {
        render(<TopicSubscriber ros={{}} topicName={null} messageType="std_msgs/msg/String" />);

        expect(screen.getByText('Type: std_msgs/msg/String')).toBeInTheDocument();
        expect(screen.getByText('Status: 🔴 Not Subscribed')).toBeInTheDocument();
    });

    it('should show not subscribed status when messageType is null', () => {
        render(<TopicSubscriber ros={{}} topicName="/test_topic" messageType={null} />);

        expect(screen.getByText('Topic: /test_topic')).toBeInTheDocument();
        expect(screen.getByText('Status: 🔴 Not Subscribed')).toBeInTheDocument();
    });

    it('should handle empty topic name', () => {
        render(<TopicSubscriber ros={null} topicName="" messageType="std_msgs/msg/String" />);

        expect(screen.getByText('Topic:')).toBeInTheDocument();
        expect(screen.getByText('Type: std_msgs/msg/String')).toBeInTheDocument();
        expect(screen.getByText('Status: 🔴 Not Subscribed')).toBeInTheDocument();
    });

    it('should handle empty message type', () => {
        render(<TopicSubscriber ros={null} topicName="/test_topic" messageType="" />);

        expect(screen.getByText('Topic: /test_topic')).toBeInTheDocument();
        expect(screen.getByText('Type:')).toBeInTheDocument();
        expect(screen.getByText('Status: 🔴 Not Subscribed')).toBeInTheDocument();
    });

    it('should render component with proper styling', () => {
        const { container } = render(<TopicSubscriber ros={null} topicName="/test" messageType="test" />);

        expect(container.firstChild).toHaveStyle({
            border: '1px solid #ccc',
            borderRadius: '8px',
            padding: '16px',
            margin: '16px 0',
            backgroundColor: '#f9f9f9'
        });
    });
}); 