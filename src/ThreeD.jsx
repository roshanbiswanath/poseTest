import React, { useEffect, useRef } from 'react'
import { Box, Line, OrbitControls, Plane, QuadraticBezierLine, useHelper } from '@react-three/drei'
import { useFrame, useThree } from '@react-three/fiber'
import * as poseDetection from '@tensorflow-models/pose-detection';
import { Vector3, AxesHelper, Quaternion, Euler } from 'three';
import { useLoader } from '@react-three/fiber'
import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import { Raycaster, SkeletonHelper } from 'three';
import { CCDIKSolver } from 'three/addons/animation/CCDIKSolver.js';
import { MMDLoader } from 'three/addons/loaders/MMDLoader.js';
import { MMDAnimationHelper } from 'three/addons/animation/MMDAnimationHelper.js';
import { Pose } from 'kalidokit'
// import { socket } from './socket';

let body = {}

const bind = (model, body) => {
  model.traverse(o => {
    // console.log(o)
    if (o.isBone && o.name === 'mixamorigNeck' && body.neck == null) {
      body.neck = o;
    }
    else if (o.isBone && o.name === 'mixamorigHead' && body.head == null) {
      body.head = o;
    }
    else if (o.isBone && o.name === 'mixamorigRightShoulder' && body.right_shoulder == null) {
      body.right_shoulder = o;
    }
    else if (o.isBone && o.name === 'mixamorigLeftShoulder' && body.left_shoulder == null) {
      body.left_shoulder = o;
    }

    else if (o.isBone && o.name === 'mixamorigRightArm' && body.right_arm == null) {
      body.right_arm = o;
    }
    else if (o.isBone && o.name === 'mixamorigRightForeArm' && body.right_fore_arm == null) {
      body.right_fore_arm = o;
    }
    else if (o.isBone && o.name === 'mixamorigLeftArm' && body.left_arm == null) {
      body.left_arm = o;
    }
    else if (o.isBone && o.name === 'mixamorigLeftForeArm' && body.left_fore_arm == null) {
      body.left_fore_arm = o;
    }

    else if (o.isBone && o.name === 'mixamorigRightHand' && body.right_hand == null) {
      body.right_hand = o;
    }
    else if (o.isBone && o.name === 'mixamorigLeftHand' && body.left_hand == null) {
      body.left_hand = o;
    }
    else if (o.isBone && o.name === 'mixamorigRightHandIndex4' && body.right_hand_index_4 == null) {
      body.right_hand_index_4 = o;
    }
    else if (o.isBone && o.name === 'mixamorigRightThumb4' && body.right_thumb_4 == null) {
      body.right_thumb_4 = o;
      body.right_thumb_4.removeFromParent();
      if (body.right_hand) {
        body.right_thumb_4.attach(body.right_hand);
        console.log("right_thumb_4 attached to right_hand");
      }

    }
    else if (o.isBone && o.name === 'mixamorigLeftThumb4' && body.left_thumb_4 == null) {
      body.left_thumb_4 = o;
      body.left_thumb_4.removeFromParent();
      if (body.left_hand) {
        body.left_thumb_4.attach(body.right_hand);
        console.log("left_thumb_4 attached to right_hand");
      }

    }

    else if (o.isBone && o.name === 'mixamorigRightLeg' && body.right_leg == null) {
      body.right_leg = o;
    }
    else if (o.isBone && o.name === 'mixamorigSpine1' && body.spine_1 == null) {
      body.spine_1 = o;
    }
    else if (o.isBone && o.name === 'mixamorigSpine2' && body.spine_2 == null) {
      body.spine_2 = o;
    }
    else if (o.isBone && o.name === 'mixamorigSpine' && body.spine == null) {
      body.spine = o;
    }
    else if (o.isBone && o.name === 'mixamorigHips' && body.hips == null) {
      body.hips = o;
    }
  }

  )
  // body.hips = model.children[2]
  // body.spine = body.hips.children[1]
  // body.spine_1 = body.spine.children[1]
  // body.spine_2 = body.spine_1.children[1]
  // body.right_shoulder = body.spine_2.children[1]
  // body.neck = body.spine_2.children[2]
  // body.left_shoulder = body.spine_2.children[3]
  // body.right_arm = body.right_shoulder.children[1]
  // body.right_fore_arm = body.right_arm.children[1]
  // body.right_hand = body.right_fore_arm.children[1]
  // body.right_hand_index_4 = body.right_hand.children[1]
  // body.right_thumb_4 = body.right_hand.children[2]
  // body.left_arm = body.left_shoulder.children[1]
  // body.left_fore_arm = body.left_arm.children[1]
  // body.left_hand = body.left_fore_arm.children[1]
  // body.left_hand_index_4 = body.left_hand.children[1]
  // body.left_thumb_4 = body.left_hand.children[2]
  // body.right_leg = body.hips.children[2]
  // body.left_leg = body.hips.children[3]
  // body.right_up_leg = body.right_leg.children[1]
  // body.left_up_leg = body.left_leg.children[1]
  // body.right_foot = body.right_up_leg.children[1]
  // body.left_foot = body.left_up_leg.children[1]
  // body.right_toe_base = body.right_foot.children[1]
  // body.left_toe_base = body.left_foot.children[1]
  // body.right_toe = body.right_toe_base.children[1]
  // body.left_toe = body.left_toe_base.children[1]
  // console.log(model.children[2])
  // console.log(body)
}

const updateData = (body_pose) => {

  body.body_pose = body_pose;

  // console.log(body_pose)
  if (body.neck) {
    // poseAngles(body.neck);
  }

  if (body.right_arm) {
    poseAngles(body.right_arm);
  }
  if (body.right_fore_arm) {
    poseAngles(body.right_fore_arm);
  }
  if (body.left_arm) {
    poseAngles(body.left_arm);
  }
  if (body.left_fore_arm) {
    poseAngles(body.left_fore_arm);
  }
  if (body.right_hand) {
    poseAngles(body.right_hand);
  }
  if (body.left_hand) {
    poseAngles(body.left_hand);
  }

}

const poseAngles = (joint) => {
  if (body.body_pose.length == 0) return;
  // console.log(joint.name)
  // console.log(body.body_pose)
  const pose_left_shoulder = new Vector3(-body.body_pose[11].x, -body.body_pose[11].y, -body.body_pose[11].z);
  const pose_right_shoulder = new Vector3(-body.body_pose[12].x, -body.body_pose[12].y, -body.body_pose[12].z);
  const pose_left_elbow = new Vector3(-body.body_pose[13].x, -body.body_pose[13].y, -body.body_pose[13].z);
  const pose_right_elbow = new Vector3(-body.body_pose[14].x, -body.body_pose[14].y, -body.body_pose[14].z);
  const pose_left_hand = new Vector3(-body.body_pose[15].x, -body.body_pose[15].y, -body.body_pose[15].z);
  const pose_right_hand = new Vector3(-body.body_pose[16].x, -body.body_pose[16].y, -body.body_pose[16].z);
  const pose_left_hand_thumb_4 = new Vector3(-body.body_pose[21].x, -body.body_pose[21].y, -body.body_pose[21].z);
  const pose_right_hand_thumb_4 = new Vector3(-body.body_pose[22].x, -body.body_pose[22].y, -body.body_pose[22].z);
  const pose_left_hip = new Vector3(-body.body_pose[23].x, -body.body_pose[23].y, -body.body_pose[23].z);
  const pose_right_hip = new Vector3(-body.body_pose[24].x, -body.body_pose[24].y, -body.body_pose[24].z);

  const pose_hips = (new Vector3().copy(pose_left_hip)).add(pose_right_hip).multiplyScalar(0.5);
  const pose_spine_2 = (new Vector3().copy(pose_right_shoulder)).add(pose_left_shoulder).multiplyScalar(0.5); //.multiplyScalar(0.728);
  // console.log(pose_hips,pose_spine_2)
  var point_parent;
  var point_articulation;
  var point_child;
  // if (joint == body.neck) {
  //   var point_parent = pose_hips;
  //   var point_articulation = pose_spine_2;
  //   var point_arm = pose_right_elbow;

  //   const vec_parent = new Vector3().subVectors(point_articulation, point_parent).multiplyScalar(0.375);
  //   const vec_bone = new Vector3().subVectors(point_arm, point_articulation);

  //   setJointAnglesFromVects(joint, vec_bone, vec_parent);
  // }
  if (joint == body.right_arm) {
    point_parent = pose_spine_2;
    point_articulation = pose_right_shoulder;
    point_child = pose_right_elbow;
  }

  else if (joint == body.left_arm) {
    point_parent = pose_spine_2;
    point_articulation = pose_left_shoulder;
    point_child = pose_left_elbow;
  }
  else if (joint == body.right_fore_arm) {
    point_parent = pose_right_shoulder;
    point_articulation = pose_right_elbow;
    point_child = pose_right_hand;
  }
  else if (joint == body.left_fore_arm) {
    point_parent = pose_left_shoulder;
    point_articulation = pose_left_elbow;
    point_child = pose_left_hand;
  }
  else if (joint == body.right_hand) {
    point_parent = pose_right_elbow;
    point_articulation = pose_right_hand;
    point_child = pose_right_hand_thumb_4;
  }
  else if (joint == body.left_hand) {
    point_parent = pose_left_elbow;
    point_articulation = pose_left_hand;
    point_child = pose_left_hand_thumb_4;
  }
  const vec_parent = new Vector3().subVectors(point_articulation, point_parent);
  const vec_bone = new Vector3().subVectors(point_child, point_articulation);
  setJointAnglesFromVects(joint, vec_parent, vec_bone);
}

function setJointAnglesFromVects(joint, vec_parent_world, vec_child_world) {
  const vec_child_local = joint.parent.clone().worldToLocal(vec_child_world.clone());
  const vec_parent_local = joint.parent.clone().worldToLocal(vec_parent_world.clone());
  var quat_pose_rot = new Quaternion();
  quat_pose_rot.setFromUnitVectors(vec_parent_local.clone().normalize(), vec_child_local.clone().normalize());
  joint.quaternion.rotateTowards(quat_pose_rot.clone(), 0.05);
}


function ThreeD({ cameraRef, debugRef }) {
  // let boxRef = useRef()

  const fullBody = useLoader(GLTFLoader, 'fullBody.glb')

  const stacy = useLoader(GLTFLoader, 'stacy_lightweight.glb')
  console.log(stacy)
  const fbx = useLoader(FBXLoader, 'vanguard.fbx')
  // console.log(fbx)
  let boneNameList = []
  // console.log(fbx)
  console.log(fbx)
  console.log(body)
  // var helper_axes = new AxesHelper(80);
  // body.spine.add(helper_axes);
  fbx.getObjectByName("vanguard_Mesh").skeleton.bones.map((bone, index) => {
    boneNameList.push(bone.name)
  })
  console.log(boneNameList)
  // let mmdLoader = new MMDLoader()
  // mmdLoader.load('Yahiko/Yahiko.pmx', (mesh) => {
  //   console.log(mesh)
  // })
  // const yahiko = useLoader(MMDLoader, 'Yahiko/Yahiko.pmx')
  // const helper = new MMDAnimationHelper();
  // console.log(yahiko)
  // helper.add(yahiko, {
  //   physics: false,
  //   animation: yahiko.animations[0]
  // })

  const shirt = useLoader(GLTFLoader, 'rigged.glb')
  // console.log(shirt)
  const watch = useLoader(GLTFLoader, 'watch.glb')

  let vector3 = new Vector3()
  let position = new Vector3()
  const { camera, raycaster } = useThree()
  console.log(camera)
  console.log(raycaster)
  let poseRef = useRef()
  let modelRef = useRef()
  useHelper(modelRef, SkeletonHelper)
  let planeRef = useRef()
  let threeDbody = useRef()
  let detector
  // console.log(poseDetection.calculators.keypointsToNormalizedKeypoints)
  const runEstimation = async () => {
    const model = poseDetection.SupportedModels.BlazePose;
    const detectorConfig = {
      runtime: 'mediapipe', // or 'tfjs'
      modelType: 'full',
      solutionPath: 'https://cdn.jsdelivr.net/npm/@mediapipe/pose',
    };
    detector = await poseDetection.createDetector(model, detectorConfig);
    console.log(detector)
    let poses = await detector.estimatePoses(cameraRef.current.video);
    // detector.detectForVideo(cameraRef.current.video)
    // detector.onResults((results) => {
    //   console.log(results)
    // })
    console.log(detector.poseSolution)
    console.log(cameraRef.current)
    setInterval(() => {
      getPoses(detector)
      // socket.emit("predict", cameraRef.current.getScreenshot())
      // console.log(JSON.stringify(poseRef.current))
    }, 1000 / 10)
  }

  const printPoses = async (detector) => {
    if (cameraRef.current == null) return
    if (cameraRef.current.video == null) return
    poses = await detector.estimatePoses(cameraRef.current.video);
    // poseRef.current = poses
    // console.log(poses) // print all
    return poses
  }

  const getPoses = async (detector) => {
    if (detector == null) return
    if (cameraRef.current == null) return
    if (cameraRef.current.video == null) return
    let poses = await detector.estimatePoses(cameraRef.current.video);
    poseRef.current = poses
    poseRef.current[0].keypoints = poseDetection.calculators.keypointsToNormalizedKeypoints(poseRef.current[0].keypoints, { width: 1280, height: 960 })
  }


  let solver
  let bodyRefs = []
  for (let i = 0; i < 33; i++) {
    bodyRefs.push(useRef())
  }

  let bodyRefs2D = []
  for (let i = 0; i < 33; i++) {
    bodyRefs2D.push(useRef())
  }

  let boneRefs3D = []
  boneRefs3D[0] = []
  boneRefs3D[1] = []
  for (let i = 0; i < 35; i++) {
    boneRefs3D[1].push(useRef())
  }

  boneRefs3D[0] = [[0, 1], [1, 2], [2, 3], [3, 7], [0, 4], [4, 5], [5, 6], [6, 8], [9, 10], [11, 12], [11, 13], [13, 15], [15, 21], [15, 17], [15, 19], [17, 19], [12, 14], [14, 16], [16, 22], [16, 18], [16, 20], [18, 20], [11, 23], [12, 24], [23, 24], [23, 25], [25, 27], [27, 29], [29, 31], [27, 31], [24, 26], [26, 28], [28, 30], [30, 32], [28, 32]]

  // console.log(boneRefs3D[0])
  useEffect(() => {
    // modelRef.current.getObjectByName("mixamorigLeftArm").rotateX(Math.PI/2)


    // const iks = [{
    //   target: 14,
    //   effector: 12,
    //   links: [ { index: 16 } ] // "bone2", "bone1", "bone0"
    // }]
    // const iks = [
    //   // Left Arm IK Chain
    //   {
    //     target: 10, // Left Hand
    //     effector: 8, // Left Arm
    //     links: [
    //       { index: 9 }, // Left Forearm
    //       { index: 7 }, // Left Shoulder
    //       { index: 5 }, // Neck
    //       { index: 4 }, // Head
    //     ]
    //   },

    //   // Right Arm IK Chain
    //   {
    //     target: 34, // Right Hand
    //     effector: 32, // Right Arm
    //     links: [
    //       { index: 33 }, // Right Forearm
    //       { index: 31 }, // Right Shoulder
    //       { index: 5 }, // Neck
    //       { index: 4 }, // Head
    //     ]
    //   },

    //   // Left Leg IK Chain
    //   {
    //     target: 56, // Right Foot
    //     effector: 55, // Right Leg
    //     links: [
    //       { index: 54 }, // Right Up Leg
    //       { index: 0 }, // Hips
    //     ]
    //   },

    //   // Right Leg IK Chain
    //   {
    //     target: 60, // Left Foot
    //     effector: 59, // Left Leg
    //     links: [
    //       { index: 58 }, // Left Up Leg
    //       { index: 0 }, // Hips
    //     ]
    //   },

    //   // Spine IK Chain
    //   {
    //     target: 5, // Head
    //     effector: 0, // Hips
    //     links: [
    //       { index: 4 }, // Neck
    //       { index: 3 }, // Spine 2
    //       { index: 2 }, // Spine 1
    //       { index: 1 }, // Spine
    //     ]
    //   },

    //   // Add more IK chains for other body parts as needed
    // ];



    // solver = new CCDIKSolver( modelRef.current.getObjectByName("vanguard_Mesh"), iks );
    // socket.on('connect', console.log(socket.id))
    runEstimation()
    bind(stacy.scene, body)

    bodyRefs2D.map((bodyRef, index) => {
      bodyRef.current.material.transparent = true
    })
    // planeRef.current.material.transparent = true
    // planeRef.current.material.opacity = 0.2

    boneRefs3D[1].map((boneRef, index) => {
      // console.log(index)
      // console.log(boneRef)
      boneRef.current.material.transparent = true
      boneRef.current.material.opacity = 0.5
      // console.log(boneRef.current)
      boneRef.current.geometry.setFromPoints([bodyRefs[boneRefs3D[0][index][0]].current.position, bodyRefs[boneRefs3D[0][index][1]].current.position])
      boneRef.current.geometry.verticesNeedUpdate = true
      // boneRefs3D[1][index].current.geometry.setFromPoints([bodyRefs[boneRefs3D[0][index][0]].current.position, bodyRefs[boneRefs3D[0][index][1]].current.position])
    })

    // console.log(modelRef.current.getObjectByName("mixamorigRightShoulder"))
    // bodyRefs.map((bodyRef, index) => {
    //   bodyRef.current.material.transparent = true
    // })

  }, [])



  useFrame((delta) => {
    // solver?.update()
    // if(poses == null) return
    // if(poses[0] == null) return
    // if(poses[0].keypoints == null) return
    // console.log(navigator.mediaDevices.enumerateDevices())
    if (poseRef.current == null) return
    if (poseRef.current[0] == null) return
    if (poseRef.current[0].keypoints3D == null) return
    // console.log(poseRef.current[0].keypoints3D)
    debugRef.current.innerText = JSON.stringify(poseRef.current[0].keypoints3D)
    // console.log(debugRef.current)

    // updateData(poseRef.current[0].keypoints3D)

    // console.log(poseRef.current[0])
    // console.log(cameraRef.current.video.width)
    // camera.position.copy(bodyRefs2D[0].current.position).add(new Vector3(0, 0, 500))
    // console.log(poseDetection.calculators.keypointsToNormalizedKeypoints(poseRef.current[0].keypoints,{width:1280,height:960}))

    let scale = bodyRefs2D[5].current.position.distanceTo(bodyRefs2D[2].current.position)
    // console.log(scale)

    for (let i = 0; i < 32; i++) {
      if (poseRef.current[0].keypoints[i] == null) return
      if (poseRef.current[0].keypoints[i].x == null) return
      if (poseRef.current[0].keypoints[i].y == null) return
      if (poseRef.current[0].keypoints[i].z == null) return
      vector3.set(poseRef.current[0].keypoints[i].x - 0.5, poseRef.current[0].keypoints[i].y - 0.5, 0)
      // console.log(vector3)
      vector3.multiplyScalar(-2)
      vector3.unproject(camera)

      // console.log(vector3)
      bodyRefs2D[i].current.position.copy(vector3)

      // let norm = vector3.sub(camera.position).normalize()
      // let ray = new Raycaster(camera.position, norm)
      // let intersects = ray.intersectObject(planeRef.current)
      // console.log(intersects)
      // bodyRefs2D[i].current.position.copy(intersects[0].point)

      // console.log(bodyRefs2D[i].current.position)


      bodyRefs2D[i].current.scale.setScalar(scale * 0.05)

      if (poseRef.current[0].keypoints[i].score >= 0.5) {
        bodyRefs2D[i].current.visible = true
      } else {
        bodyRefs2D[i].current.visible = false
      }
      bodyRefs2D[i].current.material.opacity = poseRef.current[0].keypoints[i].score
    }
    // let tempVector1 = new Vector3( poseRef.current[0].keypoints[12].x - 0.5, poseRef.current[0].keypoints[12].y - 0.5, 0)
    // let tempVector2 = new Vector3( poseRef.current[0].keypoints[11].x - 0.5, poseRef.current[0].keypoints[11].y - 0.5, 0)
    // tempVector1.multiplyScalar(-2)
    // tempVector2.multiplyScalar(-2)
    // tempVector1.unproject(camera)
    // tempVector2.unproject(camera)
    // let shirtVector = new Vector3((tempVector1.x + tempVector2.x)/2, (tempVector1.y + tempVector2.y)/2, (tempVector1.z + tempVector2.z)/2)
    // let shirtScale = tempVector1.distanceTo(tempVector2)

    let leftHip = new Vector3(poseRef.current[0].keypoints[23].x - 0.5, poseRef.current[0].keypoints[23].y - 0.5, 0)
    let rightHip = new Vector3(poseRef.current[0].keypoints[24].x - 0.5, poseRef.current[0].keypoints[24].y - 0.5, 0)
    leftHip.multiplyScalar(-2)
    rightHip.multiplyScalar(-2)
    leftHip.unproject(camera)
    rightHip.unproject(camera)
    let hipCenter = new Vector3((leftHip.x + rightHip.x) / 2, (leftHip.y + rightHip.y) / 2, (leftHip.z + rightHip.z) / 2)
    // hipCenter.z = 0
    threeDbody.current.position.copy(hipCenter)


    let leftShoulder = new Vector3(poseRef.current[0].keypoints[11].x - 0.5, poseRef.current[0].keypoints[11].y - 0.5, 0)
    let rightShoulder = new Vector3(poseRef.current[0].keypoints[12].x - 0.5, poseRef.current[0].keypoints[12].y - 0.5, 0)
    leftShoulder.multiplyScalar(-2)
    rightShoulder.multiplyScalar(-2)
    leftShoulder.unproject(camera)
    rightShoulder.unproject(camera)
    let shoulderCenter = new Vector3((leftShoulder.x + rightShoulder.x) / 2, (leftShoulder.y + rightShoulder.y) / 2, (leftShoulder.z + rightShoulder.z) / 2)

    let bodyScale = shoulderCenter.distanceTo(hipCenter)

    threeDbody.current.scale.setScalar(bodyScale * 2.01)

    // threeDbody.current.position.copy()

    // modelRef.current.scale.setScalar(shirtScale*2.6)
    // modelRef.current.position.copy(shirtVector)
    // modelRef.current.lookAt(bodyRefs2D[0].current.position)


    // modelRef.current.position.copy(bodyRefs2D[0].current.position)



    // let solved = Pose.solve(poseRef.current[0].keypoints3D,poseRef.current[0].keypoints,{runtime:'mediapipe',video:cameraRef.current});
    // console.log(solved)

    // modelRef.current.getObjectByName("vanguard_Mesh").skeleton.bones[34].rotation.x = solved.RightHand.x;
    // modelRef.current.getObjectByName("vanguard_Mesh").skeleton.bones[34].rotation.y = solved.RightHand.y;
    // modelRef.current.getObjectByName("vanguard_Mesh").skeleton.bones[34].rotation.z = solved.RightHand.z;

    for (let i = 0; i < 32; i++) {
      if (poseRef.current[0].keypoints3D[i] == null) return
      if (poseRef.current[0].keypoints3D[i].x == null) return
      if (poseRef.current[0].keypoints3D[i].y == null) return
      if (poseRef.current[0].keypoints3D[i].z == null) return
      bodyRefs[i].current.position.x = -1 * poseRef.current[0].keypoints3D[i].x
      bodyRefs[i].current.position.y = -1 * poseRef.current[0].keypoints3D[i].y
      bodyRefs[i].current.position.z = -1 * poseRef.current[0].keypoints3D[i].z
      // bodyRefs[i].current.scale.setScalar(0.1)
      bodyRefs[i].current.material.opacity = poseRef.current[0].keypoints[i].score
      // console.log(bodyRefs[i].current.position)
    }

    // if(poseRef.current[0].keypoints3D[0] == null) return
    // if(poseRef.current[0].keypoints3D[0].x == null) return
    // if(poseRef.current[0].keypoints3D[0].y == null) return
    // if(poseRef.current[0].keypoints3D[0].z == null) return
    // boxRef.current.position.x = poseRef.current[0].keypoints3D[2].x*5
    // boxRef.current.position.y = poseRef.current[0].keypoints3D[2].y*5
    // boxRef.current.position.z = poseRef.current[0].keypoints3D[2].z*5


    // console.log(modelRef.current.getObjectByName("vanguard_Mesh").skeleton.bones)
    // console.log(bodyRefs[16].current.position)

    // Binding
    // updateData(poseRef.current[0].keypoints3D)

    // position.copy(bodyRefs[15].current.position)
    // let tempPosition = modelRef.current.getObjectByName("vanguard_Mesh").skeleton.bones[34].worldToLocal(position)
    // let handPosition = position.set(bodyRefs[14].current.position.x - bodyRefs[16].current.position.x,bodyRefs[14].current.position.y - bodyRefs[16].current.position.y,bodyRefs[14].current.position.z - bodyRefs[16].current.position.z)
    // modelRef.current.getObjectByName("vanguard_Mesh").skeleton.bones[34].position.copy(handPosition)
    // console.log(bodyRefs[16].current.getWorldPosition())
    // console.log(modelRef.current.getObjectByName("vanguard_Mesh").skeleton.bones)


    // bodyRefs[15].current.getWorldPosition(position)
    // let localPos = modelRef.current.getObjectByName("stacy").skeleton.bones[34].worldToLocal(position)
    // console.log(localPos)
    // modelRef.current.getObjectByName("stacy").skeleton.bones[34].position.set(localPos.x, localPos.y, localPos.z)
    // console.log(localPos)


    // let leftShoulderEst = new Vector3(-poseRef.current[0].keypoints3D[12].x, -poseRef.current[0].keypoints3D[12].y, -poseRef.current[0].keypoints3D[12].z)
    // let leftElbEst = new Vector3(-poseRef.current[0].keypoints3D[14].x, -poseRef.current[0].keypoints3D[14].y, -poseRef.current[0].keypoints3D[14].z)
    // let leftHandEst = new Vector3(-poseRef.current[0].keypoints3D[16].x, -poseRef.current[0].keypoints3D[16].y, -poseRef.current[0].keypoints3D[16].z)

    // let leftShoulderModel = new Vector3().copy(modelRef.current.getObjectByName("mixamorigLeftArm").position)
    // let leftElbModel = new Vector3().copy(modelRef.current.getObjectByName("mixamorigLeftForeArm").position)
    // let leftHandModel = new Vector3().copy(modelRef.current.getObjectByName("mixamorigLeftHand").position)

    // let leftShoulderModelGlobal = new Vector3().copy(leftShoulderModel);
    // console.log(leftShoulderModelGlobal)
    // modelRef.current.getObjectByName("mixamorigLeftArm").localToWorld(leftShoulderModelGlobal)
    // console.log(leftShoulderModelGlobal)
    // modelRef.current.getObjectByName("mixamorigLeftArm").position.copy(leftShoulderModelGlobal)

    // let leftElbModelGlobal = new Vector3().copy(leftElbModel)
    // modelRef.current.getObjectByName("mixamorigLeftForeArm").localToWorld(leftElbModelGlobal)
    // console.log(leftElbModelGlobal)
    // modelRef.current.getObjectByName("mixamorigLeftForeArm").position.copy(leftElbModelGlobal)
    // let handPos = bodyRefs[16].current.position.clone()
    // console.log(handPos)
    // modelRef.current.getObjectByName("mixamorigLeftHand").clone().worldToLocal(handPos)
    // console.log(handPos)
    // modelRef.current.getObjectByName("mixamorigLeftHand").position.copy(handPos)


    // let leftElbModelGlobal = new Vector3().copy(modelRef.current.getObjectByName("mixamorigRightForeArm").getWorldPosition())
    // let leftHandModelGlobal = new Vector3().copy(modelRef.current.getObjectByName("mixamorigRightHand").getWorldPosition())

    // console.log(leftShoulderEst, leftElbEst, leftHandEst)
    // console.log(leftShoulderModel, leftElbModel, leftHandModel)
    // console.log(leftShoulderModelGlobal, leftElbModelGlobal, leftHandModelGlobal)



    //Stacy Movement

    /*
    // Angle Try
    // Arm Joint
    let leftArmJoint = modelRef.current.getObjectByName("mixamorigLeftArm")
    let leftForeArmJoint = modelRef.current.getObjectByName("mixamorigLeftForeArm")
    // console.log(leftArmJoint.position.clone())
    // let leftArmJointPosGlobal = leftArmJoint.localToWorld(leftArmJoint.position.clone()).clone()
    // let leftForeArmJointPosGlobal = leftForeArmJoint.localToWorld(leftForeArmJoint.position.clone()).clone()
    // let curArmDir = new Vector3().subVectors(leftForeArmJointPosGlobal,leftArmJointPosGlobal).normalize()
    // console.log(curArmDir)

    let curArmEst = new Vector3(10,10,10)
    let curArmDir = new Vector3()
    let leftArmEst = bodyRefs[12].current.position.clone()
    let leftForeArmEst = bodyRefs[14].current.position.clone()
    let leftHandEst = bodyRefs[16].current.position.clone()

    curArmDir.subVectors(leftForeArmEst,leftArmEst).normalize()
    curArmEst.subVectors(leftHandEst,leftForeArmEst).normalize()

    let quat = new Quaternion();
    quat.setFromUnitVectors(curArmDir,curArmEst);
    leftForeArmJoint.quaternion.rotateTowards(quat.clone(),0.05)
    
    let rightArmEst = bodyRefs[11].current.position.clone()
    curArmDir.subVectors(leftArmEst,rightArmEst).normalize()
    curArmEst.subVectors(leftForeArmEst,leftArmEst).normalize()
    let quat2 = new Quaternion();
    quat2.setFromAxisAngle(curArmDir,-Math.PI)
    quat.setFromUnitVectors(curArmDir,curArmEst);
    // quat.multiply(quat2)
    leftArmJoint.quaternion.rotateTowards(quat.clone(),0.1)

    let rightForeArmEst = bodyRefs[13].current.position.clone()
    let rightHandEst = bodyRefs[15].current.position.clone()
    curArmDir.subVectors(rightForeArmEst,rightArmEst).normalize()
    curArmEst.subVectors(rightHandEst,rightForeArmEst).normalize()
    quat.setFromUnitVectors(curArmDir,curArmEst);
    modelRef.current.getObjectByName("mixamorigRightForeArm").quaternion.rotateTowards(quat.clone(),0.1)
    curArmDir.subVectors(rightArmEst,leftArmEst).normalize()
    curArmEst.subVectors(rightForeArmEst,rightArmEst).normalize()
    quat.setFromUnitVectors(curArmDir,curArmEst);
    modelRef.current.getObjectByName("mixamorigRightArm").quaternion.rotateTowards(quat.clone(),0.1)

    */

    // Stacy End



    // Mediapipe Styled Bone Trial rigged.glb
    /*
    let leftArmJoint = modelRef.current.getObjectByName("Bone008")
    let leftForeArmJoint = modelRef.current.getObjectByName("Bone009")
    // console.log(leftArmJoint)

    let rightArmJoint = modelRef.current.getObjectByName("Bone007")

    let rightArmEst = bodyRefs[12].current.position.clone()
    let rightForeArmEst = bodyRefs[14].current.position.clone()
    let rightHipEst = bodyRefs[24].current.position.clone()

    let armForeArmDirEst = new Vector3().subVectors(rightForeArmEst, rightArmEst).normalize()
    let armHipDirEst = new Vector3().subVectors(rightHipEst, rightArmEst).normalize()

    let quat = new Quaternion();
    quat.setFromUnitVectors(armHipDirEst, armForeArmDirEst);
    rightArmJoint.setRotationFromQuaternion(quat.clone().conjugate())
    rightArmJoint.rotateZ(Math.PI)


    let leftArmEst = bodyRefs[11].current.position.clone()
    let leftForeArmEst = bodyRefs[13].current.position.clone()
    let leftHipEst = bodyRefs[25].current.position.clone()

    armForeArmDirEst = new Vector3().subVectors(leftForeArmEst, leftArmEst).normalize().negate()
    armHipDirEst = new Vector3().subVectors(leftHipEst, leftArmEst).normalize().negate()

    // let leftArmJointPosGlobal = leftArmJoint.position.clone()
    // let leftForeArmJointPosGlobal = leftForeArmJoint.position.clone()
    // leftArmJoint.localToWorld(leftArmJointPosGlobal)
    // leftForeArmJoint.localToWorld(leftForeArmJointPosGlobal)

    // let armForeArmDirEst = new Vector3().subVectors(leftForeArmEst, leftArmEst).normalize()
    // let armForeArmDirJoint = new Vector3().subVectors( leftForeArmJointPosGlobal ,leftArmJointPosGlobal).normalize()

    quat.setFromUnitVectors(armHipDirEst, armForeArmDirEst);
    leftArmJoint.setRotationFromQuaternion(quat.clone())
    leftArmJoint.rotateZ(Math.PI)
    // leftArmJoint.quaternion.rotateTowards(quat.clone(), 0.05)

    */


    // leftArmJoint.rotateX(Math.PI/4)



    // setJointAnglesFromVects(leftForeArmJoint,curArmDir,curArmEst)

    // function setJointAnglesFromVects(joint, vec_parent_world, vec_child_world) {
    //   const vec_child_local = joint.parent.clone().worldToLocal(vec_child_world.clone());
    //   const vec_parent_local = joint.parent.clone().worldToLocal(vec_parent_world.clone());
    //   var quat_pose_rot = new Quaternion();
    //   quat_pose_rot.setFromUnitVectors(vec_parent_local.clone().normalize(), vec_child_local.clone().normalize());
    //   joint.quaternion.rotateTowards(quat_pose_rot.clone(), 0.05);
    // }



    // Mediapipe styled bone fullBody.glb

    let quat = new Quaternion();

    let rightArmJoint = modelRef.current.getObjectByName("upper_armR")
    let rightArmEst = bodyRefs[11].current.position.clone()

    let rightForeArmEst = bodyRefs[13].current.position.clone()
    let rightHipEst = bodyRefs[23].current.position.clone()

    let armForeArmDirEst = new Vector3().subVectors(rightForeArmEst, rightArmEst).normalize()
    let armHipDirEst = new Vector3().subVectors(rightHipEst, rightArmEst).normalize().negate()

    quat.setFromUnitVectors(armHipDirEst, armForeArmDirEst);
    rightArmJoint.setRotationFromQuaternion(quat.clone().conjugate())
    // rightArmJoint.rotateY(Math.PI/2)
    rightArmJoint.rotateOnWorldAxis(armHipDirEst, Math.PI / 2)


    console.log(rightArmEst)
    // rightArmJoint.position.set(0, 0, 0)

    let leftArmJoint = modelRef.current.getObjectByName("upper_armL")
    let leftArmEst = bodyRefs[12].current.position.clone()
    let leftForeArmEst = bodyRefs[14].current.position.clone()
    let leftHipEst = bodyRefs[24].current.position.clone()

    armForeArmDirEst = new Vector3().subVectors(leftForeArmEst, leftArmEst).normalize()
    armHipDirEst = new Vector3().subVectors(leftHipEst, leftArmEst).normalize().negate()

    quat.setFromUnitVectors(armHipDirEst, armForeArmDirEst);
    leftArmJoint.setRotationFromQuaternion(quat.clone().conjugate())
    // leftArmJoint.rotateY(Math.PI/2)
    leftArmJoint.rotateOnWorldAxis(armHipDirEst, -Math.PI / 2)

    let leftElbJoint = modelRef.current.getObjectByName("forearmL")
    let leftWristEst = bodyRefs[16].current.position.clone()

    let wristElbEst = new Vector3().subVectors(leftWristEst, leftForeArmEst).normalize()
    quat.setFromUnitVectors(armForeArmDirEst,wristElbEst)
    leftElbJoint.setRotationFromQuaternion(quat.clone().conjugate())
    // leftElbJoint.rotateOnWorldAxis(armForeArmDirEst, -Math.PI / 2)

    let rightElbJoint = modelRef.current.getObjectByName("forearmR")
    let rightWristEst = bodyRefs[15].current.position.clone()

    armForeArmDirEst = new Vector3().subVectors(rightForeArmEst, rightArmEst).normalize()

    wristElbEst = new Vector3().subVectors(rightWristEst, rightForeArmEst).normalize()
    quat.setFromUnitVectors(armForeArmDirEst,wristElbEst)
    rightElbJoint.setRotationFromQuaternion(quat.clone().conjugate())
    // rightElbJoint.rotateOnWorldAxis(armForeArmDirEst, -Math.PI / 2)



    for (let i = 0; i < 35; i++) {
      if (poseRef.current[0].keypoints3D[i] == null) return
      if (poseRef.current[0].keypoints3D[i].x == null) return
      if (poseRef.current[0].keypoints3D[i].y == null) return
      if (poseRef.current[0].keypoints3D[i].z == null) return
      boneRefs3D[1][i].current.geometry.setFromPoints([bodyRefs[boneRefs3D[0][i][0]].current.position, bodyRefs[boneRefs3D[0][i][1]].current.position])
      // boneRefs3D[1][i].current.setPoints(bodyRefs[boneRefs3D[0][i][0]].current.position, bodyRefs[boneRefs3D[0][i][1]].current.position)
      // boneRefs3D[1][i].current.material.opacity = bodyRefs[boneRefs3D[0][i][0]].current.position.z 
      boneRefs3D[1][i].current.geometry.verticesNeedUpdate = true
    }


    helper.update(clock.getDelta());

    // modelRef.current.getObjectByName("mixamorigRightHand").position = bodyRefs[16].current.position

    // modelRef.current.position.copy(bodyRefs[0].current.position)

  })

  return (
    <>
      <ambientLight />
      <directionalLight position={[10, 10, 5]} />
      <pointLight position={[10, 10, 10]} />
      {/* <Plane ref={planeRef} args={[10000, 10000]} position={[0, 0, 0]} /> */}
      {/* <Box ref={boxRef} position={[0, 0, 0]} scale={[0.01,0.01,0.01]} /> */}
      {/* <mesh
        position={[0, 0, 0]}
        rotation={[0, 0, 0]}
        ref={boxRef}
      >
        <boxGeometry args={[1, 1, 1]} />
        <meshStandardMaterial color={'orange'} />
      </mesh> */}
      <group ref={threeDbody} >
        {
          bodyRefs.map((bodyRef, index) => {
            return (
              <mesh
                key={index}
                position={[0, 0, 0]}
                rotation={[0, 0, 0]}
                scale={[0.01, 0.01, 0.01]}
                ref={bodyRef}
              >
                <boxGeometry args={[1, 1, 1]} />
                <meshStandardMaterial color={'orange'} />
              </mesh>
            )
          })
        }{
          boneRefs3D[1].map((boneRef, index) => {
            return (<>
              <line key={index} ref={boneRef}>
                <bufferGeometry attach="geometry" />
                <lineBasicMaterial attach="material" color="red" linewidth={1} />
              </line>
              {/* <QuadraticBezierLine key={index+10000000} points={[[0,0,0],[0,0,0]]} ref={boneRef} lineWidth={10}></QuadraticBezierLine> */}
            </>
            )
          })
        }
        <primitive scale={[0.8, 0.8, 0.8]} position={[0, 0, 0]} object={fullBody.scene} ref={modelRef} />
      </group>
      {
        bodyRefs2D.map((bodyRef, index) => {
          return (
            <mesh
              key={index}
              position={[0, 0, 0]}
              rotation={[0, 0, 0]}
              scale={[0.01, 0.01, 0.01]}
              ref={bodyRef}
            >
              {/* <boxGeometry args={[1, 1, 1]} /> */}
              {
                // index == 15 ?
                // <primitive scale={[5, 5, 5]} object={watch.scene} position={[0, 0, 0]} rotation={[Math.PI / 2, 0, 0]} /> :
                <sphereGeometry args={[1, 32, 32]} />

              }
              <meshStandardMaterial color={'red'} />
            </mesh>
          )
        })
      }


      {/* <primitive ref={modelRef} object={yahiko} position={[0, 0, 0]} rotation={[0 , 0, 0]} scale={[2,2,2]} /> */}
      <OrbitControls />
    </>
  )
}

export default ThreeD