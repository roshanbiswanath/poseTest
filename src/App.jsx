import { useState } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import { useRef, useEffect } from 'react'
import './App.css'
import Webcam from "react-webcam";
import * as poseDetection from '@tensorflow-models/pose-detection';
import '@mediapipe/pose';
import ThreeD from './ThreeD'
import { Canvas, useFrame } from '@react-three/fiber'
import { Box } from '@react-three/drei'

function App() {
  let cameraRef = useRef()
  let canvasRef = useRef()
  let debugRef = useRef()
  let boxRef = useRef()
  let poseRef = useRef()
  let poses;

  const videoConstraints = {
    // facingMode: "user",
    // deviceId: "aac1b6934d79841b89676acaa6debb8a40e6667a41b50d3dac8feaee5380027e",
    width: 1280,
    height: 960,
  };

  useEffect(() => {
    // runEstimation()
    console.log(cameraRef.current)
    console.log(canvasRef.current)


    // canvasRef.current.width = cameraRef.current.width
    // canvasRef.current.height = cameraRef.current.height
  }, [])

  return (
    <>
      <Webcam ref={cameraRef} videoConstraints={videoConstraints} className='webcam' screenshotFormat="image/jpeg" />
      <div className='canvas-container'>
        <Canvas ref={canvasRef} camera={{ near: 0.1, far: 999999 }} >
          <ThreeD debugRef={debugRef} cameraRef={cameraRef} />
          {/* <Box ref={boxRef}></Box> */}
        </Canvas>
      </div>
      <div ref={debugRef} className='debug'>

      </div>
    </>
  )
}

export default App
