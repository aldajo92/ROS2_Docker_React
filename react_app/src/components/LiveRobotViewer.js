import React, { useRef, useState, useEffect } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Environment } from '@react-three/drei';
import ROSLIB from 'roslib';

// Live robot model that responds to ROS joint states
function LiveRobotModel({ ros, jointStates, connected }) {
    const robotRef = useRef();
    const lidarRef = useRef();

    // Simulate LiDAR rotation based on real data or time
    useFrame((state) => {
        if (lidarRef.current) {
            // Rotate LiDAR continuously to simulate scanning
            lidarRef.current.rotation.z += 0.02;
        }

        // Optional: slight float animation when not connected
        if (robotRef.current && !connected) {
            robotRef.current.position.y = Math.sin(state.clock.elapsedTime) * 0.01;
        }
    });

    return (
        <group ref={robotRef}>
            {/* Base platform */}
            <mesh position={[0, 0, 0]}>
                <boxGeometry args={[0.18, 0.12, 0.03]} />
                <meshStandardMaterial
                    color={connected ? "#404040" : "#604040"}
                    transparent={!connected}
                    opacity={connected ? 1.0 : 0.7}
                />
            </mesh>

            {/* Top shell */}
            <mesh position={[0, 0, 0.04]}>
                <boxGeometry args={[0.16, 0.10, 0.02]} />
                <meshStandardMaterial
                    color={connected ? "#606060" : "#806060"}
                    transparent={!connected}
                    opacity={connected ? 1.0 : 0.7}
                />
            </mesh>

            {/* Front bumper */}
            <mesh position={[0.09, 0, 0.02]}>
                <boxGeometry args={[0.01, 0.08, 0.02]} />
                <meshStandardMaterial color={connected ? "#303030" : "#503030"} />
            </mesh>

            {/* Back bumper */}
            <mesh position={[-0.09, 0, 0.02]}>
                <boxGeometry args={[0.01, 0.08, 0.02]} />
                <meshStandardMaterial color={connected ? "#303030" : "#503030"} />
            </mesh>

            {/* Wheels - could be animated based on wheel joint states */}
            {/* Front Left Wheel */}
            <mesh position={[0.065, 0.085, -0.02]}>
                <cylinderGeometry args={[0.04, 0.04, 0.02]} />
                <meshStandardMaterial color={connected ? "#202020" : "#402020"} />
            </mesh>

            {/* Front Right Wheel */}
            <mesh position={[0.065, -0.085, -0.02]}>
                <cylinderGeometry args={[0.04, 0.04, 0.02]} />
                <meshStandardMaterial color={connected ? "#202020" : "#402020"} />
            </mesh>

            {/* Back Left Wheel */}
            <mesh position={[-0.065, 0.085, -0.02]}>
                <cylinderGeometry args={[0.04, 0.04, 0.02]} />
                <meshStandardMaterial color={connected ? "#202020" : "#402020"} />
            </mesh>

            {/* Back Right Wheel */}
            <mesh position={[-0.065, -0.085, -0.02]}>
                <cylinderGeometry args={[0.04, 0.04, 0.02]} />
                <meshStandardMaterial color={connected ? "#202020" : "#402020"} />
            </mesh>

            {/* LiDAR sensor - animated */}
            <group ref={lidarRef}>
                <mesh position={[0, 0, 0.08]}>
                    <cylinderGeometry args={[0.025, 0.025, 0.03]} />
                    <meshStandardMaterial
                        color={connected ? "#1a1a1a" : "#3a1a1a"}
                        emissive={connected ? "#001100" : "#110000"}
                    />
                </mesh>

                {/* LiDAR top */}
                <mesh position={[0, 0, 0.095]}>
                    <cylinderGeometry args={[0.02, 0.02, 0.005]} />
                    <meshStandardMaterial
                        color={connected ? "#333" : "#533"}
                        emissive={connected ? "#002200" : "#220000"}
                    />
                </mesh>
            </group>

            {/* OLED Screen */}
            <mesh position={[0.085, 0, 0.05]}>
                <boxGeometry args={[0.005, 0.03, 0.02]} />
                <meshStandardMaterial
                    color={connected ? "#001f3f" : "#1f001f"}
                    emissive={connected ? "#001122" : "#110011"}
                />
            </mesh>

            {/* Circuit board representation */}
            <mesh position={[0, 0, 0.025]}>
                <boxGeometry args={[0.08, 0.06, 0.002]} />
                <meshStandardMaterial
                    color={connected ? "#0a5d0a" : "#5d0a0a"}
                    emissive={connected ? "#001100" : "#110000"}
                />
            </mesh>

            {/* Status indicators */}
            <mesh position={[0.02, 0.02, 0.026]}>
                <boxGeometry args={[0.01, 0.008, 0.003]} />
                <meshStandardMaterial
                    color={connected ? "#8B4513" : "#8B1343"}
                    emissive={connected ? "#110000" : "#220000"}
                />
            </mesh>

            <mesh position={[-0.02, -0.02, 0.026]}>
                <boxGeometry args={[0.012, 0.01, 0.003]} />
                <meshStandardMaterial
                    color={connected ? "#4169E1" : "#E14169"}
                    emissive={connected ? "#000011" : "#110000"}
                />
            </mesh>

            {/* Battery indicator */}
            <mesh position={[-0.06, 0, 0.026]}>
                <boxGeometry args={[0.02, 0.01, 0.005]} />
                <meshStandardMaterial
                    color={connected ? "#FFD700" : "#FF7700"}
                    emissive={connected ? "#221100" : "#221100"}
                />
            </mesh>

            {/* Connection status indicators */}
            <mesh position={[0, 0.055, 0.025]}>
                <boxGeometry args={[0.015, 0.005, 0.008]} />
                <meshStandardMaterial
                    color={connected ? "#708090" : "#907080"}
                    emissive={connected ? "#001100" : "#110000"}
                />
            </mesh>

            <mesh position={[0, -0.055, 0.025]}>
                <boxGeometry args={[0.015, 0.005, 0.008]} />
                <meshStandardMaterial
                    color={connected ? "#708090" : "#907080"}
                    emissive={connected ? "#001100" : "#110000"}
                />
            </mesh>
        </group>
    );
}

// Main Live Robot Viewer Component
const LiveRobotViewer = ({
    ros,
    connected,
    height = "500px",
    showGrid = true,
    showEnvironment = true
}) => {
    const [viewerError, setViewerError] = useState(null);
    const [isInitialized, setIsInitialized] = useState(false);
    const [jointStates, setJointStates] = useState({});
    const [robotDescription, setRobotDescription] = useState(null);
    const [lastMessageTime, setLastMessageTime] = useState(null);

    // Subscribe to joint states
    useEffect(() => {
        if (!ros || !connected) return;

        console.log('🔗 Setting up ROS subscriptions...');

        // Subscribe to joint states
        const jointStatesTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/joint_states',
            messageType: 'sensor_msgs/msg/JointState'
        });

        jointStatesTopic.subscribe((message) => {
            console.log('📊 Joint states received:', message);
            setJointStates(message);
            setLastMessageTime(new Date());
        });

        // For now, don't try to access robot_description parameter
        // This was causing JSON parsing errors
        console.log('🤖 Robot description access disabled to avoid JSON parsing errors');
        setRobotDescription('live_mode');

        return () => {
            try {
                jointStatesTopic.unsubscribe();
                console.log('🔌 Unsubscribed from ROS topics');
            } catch (error) {
                console.warn('Error unsubscribing from topics:', error);
            }
        };
    }, [ros, connected]);

    const handleCanvasError = (error) => {
        console.error('Canvas error:', error);
        setViewerError('Failed to initialize 3D viewer. Please check WebGL support.');
    };

    const handleCanvasCreated = () => {
        setIsInitialized(true);
        console.log('3D Canvas initialized successfully');
    };

    if (viewerError) {
        return (
            <div style={{
                height: height,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                backgroundColor: '#f5f5f5',
                border: '1px solid #ddd',
                borderRadius: '8px',
                color: '#666',
                flexDirection: 'column'
            }}>
                <div style={{ textAlign: 'center' }}>
                    <h3>⚠️ 3D Viewer Error</h3>
                    <p>{viewerError}</p>
                    <p>Please ensure your browser supports WebGL</p>
                    <button
                        onClick={() => setViewerError(null)}
                        style={{
                            marginTop: '12px',
                            padding: '8px 16px',
                            borderRadius: '4px',
                            border: '1px solid #007bff',
                            backgroundColor: '#007bff',
                            color: 'white',
                            cursor: 'pointer'
                        }}
                    >
                        Retry
                    </button>
                </div>
            </div>
        );
    }

    return (
        <div style={{ height: height, width: '100%', position: 'relative' }}>
            {/* Connection Status Overlay */}
            <div style={{
                position: 'absolute',
                top: '10px',
                left: '10px',
                zIndex: 10,
                padding: '8px 12px',
                backgroundColor: connected ? 'rgba(0, 128, 0, 0.8)' : 'rgba(128, 0, 0, 0.8)',
                color: 'white',
                borderRadius: '4px',
                fontSize: '12px',
                fontWeight: 'bold'
            }}>
                {connected ? '🟢 ROS Connected' : '🔴 ROS Disconnected'}
            </div>

            {/* Data Status */}
            {connected && (
                <div style={{
                    position: 'absolute',
                    top: '10px',
                    right: '10px',
                    zIndex: 10,
                    padding: '6px 10px',
                    backgroundColor: 'rgba(0, 0, 0, 0.7)',
                    color: 'white',
                    borderRadius: '4px',
                    fontSize: '11px'
                }}>
                    <div>🤖 Robot: {robotDescription ? 'Loaded' : 'Loading...'}</div>
                    <div>📊 Joints: {Object.keys(jointStates).length > 0 ? 'Active' : 'Waiting...'}</div>
                    {lastMessageTime && (
                        <div>⏰ Last: {lastMessageTime.toLocaleTimeString()}</div>
                    )}
                </div>
            )}

            {!isInitialized && (
                <div style={{
                    position: 'absolute',
                    top: '50%',
                    left: '50%',
                    transform: 'translate(-50%, -50%)',
                    zIndex: 10,
                    color: '#666',
                    textAlign: 'center'
                }}>
                    <div>🔄 Initializing live robot viewer...</div>
                </div>
            )}

            <Canvas
                camera={{ position: [0.3, 0.3, 0.2], fov: 75 }}
                onCreated={handleCanvasCreated}
                onError={handleCanvasError}
                style={{
                    background: connected
                        ? 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'
                        : 'linear-gradient(135deg, #ea6767 0%, #a24b76 100%)'
                }}
            >
                {/* Lighting setup */}
                <ambientLight intensity={0.6} />
                <directionalLight
                    position={[5, 5, 5]}
                    intensity={0.8}
                    castShadow
                />
                <pointLight position={[-5, -5, -5]} intensity={0.4} />

                {/* Environment and Grid */}
                {showEnvironment && <Environment preset="city" />}
                {showGrid && (
                    <Grid
                        args={[10, 10]}
                        cellSize={0.05}
                        cellThickness={0.5}
                        cellColor={connected ? '#6f6f6f' : '#6f4f4f'}
                        sectionSize={0.5}
                        sectionThickness={1}
                        sectionColor={connected ? '#9d4b4b' : '#9d2b2b'}
                        fadeDistance={2}
                        fadeStrength={1}
                        followCamera={false}
                        infiniteGrid={true}
                    />
                )}

                {/* Controls */}
                <OrbitControls
                    enableDamping
                    dampingFactor={0.05}
                    enablePan={true}
                    enableZoom={true}
                    enableRotate={true}
                    maxPolarAngle={Math.PI * 0.75}
                    minDistance={0.1}
                    maxDistance={2}
                />

                {/* Live Robot Model */}
                <LiveRobotModel
                    ros={ros}
                    jointStates={jointStates}
                    connected={connected}
                />
            </Canvas>
        </div>
    );
};

export default LiveRobotViewer; 