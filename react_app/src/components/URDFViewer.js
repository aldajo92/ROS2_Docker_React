import React, { useRef, useEffect, useState, Suspense } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Environment } from '@react-three/drei';
import * as THREE from 'three';
import { STLLoader } from 'three-stdlib';
import URDFLoader from 'urdf-loader';

// Component to load and display the URDF
function URDFModel({ urdfPath, meshPath }) {
    const modelRef = useRef();
    const [robot, setRobot] = useState(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);

    useEffect(() => {
        const loader = new URDFLoader();

        // Set the working path for meshes
        loader.workingPath = meshPath;

        // Configure mesh loading with STL loader
        const manager = new THREE.LoadingManager();
        loader.manager = manager;

        // Configure STL loader for better mesh handling
        const stlLoader = new STLLoader(manager);
        loader.defaultMeshLoader = stlLoader;

        console.log('Loading URDF from:', urdfPath);
        console.log('Mesh path:', meshPath);

        // Load the URDF
        loader.load(
            urdfPath,
            (result) => {
                console.log('URDF loaded successfully:', result);
                console.log('Robot children:', result.children);

                // Log all links found
                result.traverse((child) => {
                    if (child.isGroup && child.name) {
                        console.log('Found link:', child.name);
                    }
                });

                // Center the model
                const box = new THREE.Box3().setFromObject(result);
                const center = box.getCenter(new THREE.Vector3());
                result.position.sub(center);

                // Scale the model appropriately
                const size = box.getSize(new THREE.Vector3());
                const maxDim = Math.max(size.x, size.y, size.z);
                console.log('Model dimensions:', size, 'Max:', maxDim);

                // Scale to a reasonable size for viewing
                if (maxDim > 0) {
                    const targetSize = 2; // Target max dimension
                    const scale = targetSize / maxDim;
                    result.scale.setScalar(scale);
                    console.log('Applied scale:', scale);
                }

                setRobot(result);
                setLoading(false);
            },
            (progress) => {
                console.log('Loading progress:', (progress.loaded / progress.total * 100).toFixed(1) + '%');
            },
            (error) => {
                console.error('Error loading URDF:', error);
                setError(error.message || 'Failed to load URDF model');
                setLoading(false);
            }
        );

        // Cleanup function
        return () => {
            // Clean up any resources if needed
            if (loader) {
                // Basic cleanup
                console.log('Cleaning up URDF loader');
            }
        };
    }, [urdfPath, meshPath]); // Fixed dependency array

    useFrame(() => {
        if (modelRef.current) {
            // Optional: Add subtle rotation or animation
            // modelRef.current.rotation.z += 0.001;
        }
    });

    if (loading) {
        return (
            <group>
                <mesh position={[0, 0, 0]}>
                    <boxGeometry args={[0.5, 0.1, 0.5]} />
                    <meshStandardMaterial color="orange" />
                </mesh>
                <mesh position={[0, 0.2, 0]}>
                    <sphereGeometry args={[0.1]} />
                    <meshStandardMaterial color="orange" />
                </mesh>
            </group>
        );
    }

    if (error) {
        return (
            <group>
                <mesh position={[0, 0, 0]}>
                    <boxGeometry args={[1, 0.1, 1]} />
                    <meshStandardMaterial color="red" />
                </mesh>
                <mesh position={[0, 0.2, 0]}>
                    <sphereGeometry args={[0.1]} />
                    <meshStandardMaterial color="red" />
                </mesh>
            </group>
        );
    }

    return robot ? <primitive ref={modelRef} object={robot} /> : null;
}

// Loading fallback component
function LoadingBox() {
    const meshRef = useRef();

    useFrame((state) => {
        if (meshRef.current) {
            meshRef.current.rotation.x = Math.sin(state.clock.elapsedTime) * 0.2;
            meshRef.current.rotation.y = state.clock.elapsedTime * 0.5;
        }
    });

    return (
        <mesh ref={meshRef}>
            <boxGeometry args={[0.5, 0.5, 0.5]} />
            <meshStandardMaterial color="lightblue" wireframe />
        </mesh>
    );
}

// Main URDF Viewer Component
const URDFViewer = ({
    urdfPath = "/robot_description/waver.urdf",
    meshPath = "/robot_description/meshes/",
    height = "500px",
    showGrid = true,
    showEnvironment = true
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
                camera={{ position: [2, 2, 2], fov: 75 }}
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
                    shadow-mapSize={[2048, 2048]}
                    shadow-camera-far={50}
                    shadow-camera-left={-10}
                    shadow-camera-right={10}
                    shadow-camera-top={10}
                    shadow-camera-bottom={-10}
                />
                <pointLight position={[-5, -5, -5]} intensity={0.4} />
                <hemisphereLight skyColor={0xffffbb} groundColor={0x080820} intensity={0.3} />

                {/* Environment and Grid */}
                {showEnvironment && <Environment preset="city" />}
                {showGrid && (
                    <Grid
                        args={[20, 20]}
                        cellSize={0.5}
                        cellThickness={0.5}
                        cellColor={'#6f6f6f'}
                        sectionSize={5}
                        sectionThickness={1}
                        sectionColor={'#9d4b4b'}
                        fadeDistance={30}
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
                    minDistance={0.5}
                    maxDistance={10}
                    autoRotate={false}
                    autoRotateSpeed={0.5}
                />

                {/* URDF Model */}
                <Suspense fallback={<LoadingBox />}>
                    <URDFModel
                        urdfPath={urdfPath}
                        meshPath={meshPath}
                    />
                </Suspense>
            </Canvas>
        </div>
    );
};

export default URDFViewer; 