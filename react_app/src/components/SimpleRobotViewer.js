import React, { useRef, useState } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Environment } from '@react-three/drei';

// Simple geometric robot model
function SimpleRobotModel({ animate = false }) {
    const robotRef = useRef();

    useFrame(() => {
        if (robotRef.current && animate) {
            robotRef.current.rotation.z += 0.005;
        }
    });

    return (
        <group ref={robotRef}>
            {/* Base platform */}
            <mesh position={[0, 0, 0]}>
                <boxGeometry args={[0.18, 0.12, 0.03]} />
                <meshStandardMaterial color="#404040" />
            </mesh>

            {/* Top shell */}
            <mesh position={[0, 0, 0.04]}>
                <boxGeometry args={[0.16, 0.10, 0.02]} />
                <meshStandardMaterial color="#606060" />
            </mesh>

            {/* Front bumper */}
            <mesh position={[0.09, 0, 0.02]}>
                <boxGeometry args={[0.01, 0.08, 0.02]} />
                <meshStandardMaterial color="#303030" />
            </mesh>

            {/* Back bumper */}
            <mesh position={[-0.09, 0, 0.02]}>
                <boxGeometry args={[0.01, 0.08, 0.02]} />
                <meshStandardMaterial color="#303030" />
            </mesh>

            {/* Wheels */}
            {/* Front Left Wheel */}
            <mesh position={[0.065, 0.085, -0.02]}>
                <cylinderGeometry args={[0.04, 0.04, 0.02]} />
                <meshStandardMaterial color="#202020" />
            </mesh>

            {/* Front Right Wheel */}
            <mesh position={[0.065, -0.085, -0.02]}>
                <cylinderGeometry args={[0.04, 0.04, 0.02]} />
                <meshStandardMaterial color="#202020" />
            </mesh>

            {/* Back Left Wheel */}
            <mesh position={[-0.065, 0.085, -0.02]}>
                <cylinderGeometry args={[0.04, 0.04, 0.02]} />
                <meshStandardMaterial color="#202020" />
            </mesh>

            {/* Back Right Wheel */}
            <mesh position={[-0.065, -0.085, -0.02]}>
                <cylinderGeometry args={[0.04, 0.04, 0.02]} />
                <meshStandardMaterial color="#202020" />
            </mesh>

            {/* LiDAR sensor */}
            <mesh position={[0, 0, 0.08]}>
                <cylinderGeometry args={[0.025, 0.025, 0.03]} />
                <meshStandardMaterial color="#1a1a1a" />
            </mesh>

            {/* LiDAR top */}
            <mesh position={[0, 0, 0.095]}>
                <cylinderGeometry args={[0.02, 0.02, 0.005]} />
                <meshStandardMaterial color="#333" />
            </mesh>

            {/* OLED Screen */}
            <mesh position={[0.085, 0, 0.05]}>
                <boxGeometry args={[0.005, 0.03, 0.02]} />
                <meshStandardMaterial color="#001f3f" emissive="#001122" />
            </mesh>

            {/* Circuit board representation */}
            <mesh position={[0, 0, 0.025]}>
                <boxGeometry args={[0.08, 0.06, 0.002]} />
                <meshStandardMaterial color="#0a5d0a" />
            </mesh>

            {/* Small electronic components */}
            <mesh position={[0.02, 0.02, 0.026]}>
                <boxGeometry args={[0.01, 0.008, 0.003]} />
                <meshStandardMaterial color="#8B4513" />
            </mesh>

            <mesh position={[-0.02, -0.02, 0.026]}>
                <boxGeometry args={[0.012, 0.01, 0.003]} />
                <meshStandardMaterial color="#4169E1" />
            </mesh>

            {/* Battery indicator */}
            <mesh position={[-0.06, 0, 0.026]}>
                <boxGeometry args={[0.02, 0.01, 0.005]} />
                <meshStandardMaterial color="#FFD700" />
            </mesh>

            {/* Connection ports */}
            <mesh position={[0, 0.055, 0.025]}>
                <boxGeometry args={[0.015, 0.005, 0.008]} />
                <meshStandardMaterial color="#708090" />
            </mesh>

            <mesh position={[0, -0.055, 0.025]}>
                <boxGeometry args={[0.015, 0.005, 0.008]} />
                <meshStandardMaterial color="#708090" />
            </mesh>
        </group>
    );
}

// Main Simple Robot Viewer Component
const SimpleRobotViewer = ({
    height = "500px",
    showGrid = true,
    showEnvironment = true,
    animate = false
}) => {
    const [viewerError, setViewerError] = useState(null);
    const [isInitialized, setIsInitialized] = useState(false);

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
                    <div>🔄 Initializing 3D viewer...</div>
                </div>
            )}

            <Canvas
                camera={{ position: [0.3, 0.3, 0.2], fov: 75 }}
                onCreated={handleCanvasCreated}
                onError={handleCanvasError}
                style={{ background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)' }}
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
                        cellColor={'#6f6f6f'}
                        sectionSize={0.5}
                        sectionThickness={1}
                        sectionColor={'#9d4b4b'}
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

                {/* Simple Robot Model */}
                <SimpleRobotModel animate={animate} />
            </Canvas>
        </div>
    );
};

export default SimpleRobotViewer; 