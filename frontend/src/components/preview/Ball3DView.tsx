import { forwardRef, useEffect, useImperativeHandle, useMemo, useRef } from 'react'
import type { MutableRefObject } from 'react'
import { Canvas, useThree } from '@react-three/fiber'
import { Line, OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'

import type { MachineState, PreviewPath } from '../../api/types'
import type { ViewPreset } from '../../store/appStore'
import { classifyPath, getCurrentMarker, phaseOpacity, phaseStroke, shouldRenderPath } from './previewMath'

export type Ball3DHandle = {
  fit: () => void
  reset: () => void
}

type Props = {
  paths: PreviewPath[]
  machine: MachineState | null
  filter: 'all' | 'progress'
  showTravel: boolean
  preset: ViewPreset
}

export const Ball3DView = forwardRef<Ball3DHandle, Props>(function Ball3DView(
  { paths, machine, filter, showTravel, preset },
  ref,
) {
  const controlsRef = useRef<{ reset: () => void } | null>(null)

  useImperativeHandle(ref, () => ({
    fit: () => controlsRef.current?.reset(),
    reset: () => controlsRef.current?.reset(),
  }))

  const visiblePaths = useMemo(
    () => paths.filter((path) => shouldRenderPath(path, machine, filter, showTravel)),
    [filter, machine, paths, showTravel],
  )

  return (
    <div className="ball-view">
      <Canvas camera={{ position: preset === 'front' ? [0, 0, 2.8] : [1.7, 0.95, 2.45], fov: 38 }}>
        <ScenePaths machine={machine} paths={visiblePaths} />
        <SceneCamera controlsRef={controlsRef} preset={preset} />
      </Canvas>
    </div>
  )
})

function SceneCamera({
  controlsRef,
  preset,
}: {
  controlsRef: MutableRefObject<{ reset: () => void } | null>
  preset: ViewPreset
}) {
  const controlsInnerRef = useRef<OrbitControlsImpl | null>(null)
  const { camera } = useThree()

  useEffect(() => {
    if (preset === 'front') {
      camera.position.set(0, 0, 2.8)
    } else {
      camera.position.set(1.7, 0.95, 2.45)
    }
    camera.lookAt(0, 0, 0)
    controlsInnerRef.current?.reset()
  }, [camera, preset])

  useEffect(() => {
    controlsRef.current = {
      reset: () => controlsInnerRef.current?.reset(),
    }
  }, [controlsRef])

  return (
    <>
      <color attach="background" args={['#efe8dc']} />
      <ambientLight intensity={1.1} />
      <directionalLight intensity={1.6} position={[4, 3, 5]} />
      <directionalLight intensity={0.5} position={[-3, -1.4, -3]} />
      <OrbitControls ref={controlsInnerRef} enablePan enableZoom makeDefault />
    </>
  )
}

function ScenePaths({ paths, machine }: { paths: PreviewPath[]; machine: MachineState | null }) {
  return (
    <>
      <mesh>
        <sphereGeometry args={[1, 96, 96]} />
        <meshStandardMaterial color="#f7f3eb" metalness={0.04} roughness={0.82} />
      </mesh>
      <mesh rotation={[0, Math.PI / 2, 0]}>
        <ringGeometry args={[1.001, 1.002, 128]} />
        <meshBasicMaterial color="#c7b8a0" side={THREE.DoubleSide} transparent opacity={0.22} />
      </mesh>
      {paths.map((path) => {
        const phase = classifyPath(path, machine)
        const color = phaseStroke(phase, path.kind)
        const opacity = phaseOpacity(phase, path.kind)
        const marker = getCurrentMarker(path, machine)
        return (
          <group key={path.id}>
            <Line
              color={color}
              dashed={path.kind === 'travel'}
              dashScale={6}
              lineWidth={phase === 'current' ? 2.6 : 1.4}
              opacity={opacity}
              points={path.points.map(toSphere)}
              transparent
            />
            {marker ? (
              <mesh position={toSphere(marker)}>
                <sphereGeometry args={[0.022, 18, 18]} />
                <meshStandardMaterial color="#fff5a8" emissive="#fff5a8" emissiveIntensity={1.4} />
              </mesh>
            ) : null}
          </group>
        )
      })}
      <mesh position={[0, 0, 1.01]}>
        <sphereGeometry args={[0.018, 18, 18]} />
        <meshStandardMaterial color="#f97316" />
      </mesh>
    </>
  )
}

function toSphere(point: { x: number; y: number }) {
  const radius = 1.003
  const lon = THREE.MathUtils.degToRad(point.x)
  const lat = THREE.MathUtils.degToRad(point.y)
  return new THREE.Vector3(
    radius * Math.cos(lat) * Math.sin(lon),
    radius * Math.sin(lat),
    radius * Math.cos(lat) * Math.cos(lon),
  )
}
