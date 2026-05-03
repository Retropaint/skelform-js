function SkfGenericFormatFrame(frame, anim, isReverse, isLoop) {
  const lastFrame = anim.keyframes[anim.keyframes.length - 1].frame
  if (isLoop) {
    frame %= lastFrame + 1
  }

  if (isReverse) {
    frame = lastFrame - frame
  }

  return Math.round(frame)
}

function SkfGenericTimeFrame(time, anim, isReverse, isLoop) {
  const elapsed = time / 1000
  const frametime = 1 / anim.fps
  const frame = elapsed / frametime
  return SkfGenericFormatFrame(frame, anim, isReverse, isLoop)
}

function copyArray(src, dst) {
  for (let i = 0; i < dst.length; i++) {
    src[i] = dst[i]
  }
}

function rotate(point, rot) {
  return { x: point.x * Math.cos(rot) - point.y * Math.sin(rot), y: point.x * Math.sin(rot) + point.y * Math.cos(rot), }
}

function mulv2(self, other) {
  return { x: self.x * other.x, y: self.y * other.y }
}

function mulv2f(self, otherf) {
  return { x: self.x * otherf, y: self.y * otherf }
}

function addv2(self, other) {
  return { x: self.x + other.x, y: self.y + other.y }
}

function addv2f(self, other) {
  return { x: self.x + other, y: self.y + other }
}

function subv2(self, other) {
  return { x: self.x - other.x, y: self.y - other.y }
}

function subv2f(self, otherf) {
  return { x: self.x - otherf, y: self.y - otherf }
}

function magnitude(vec) {
  return Math.sqrt((vec.x * vec.x) + (vec.y * vec.y))
}

function normalize(vec) {
  let mag = magnitude(vec);
  if (mag == 0) {
    return { x: 0, y: 0 }
  }
  return { x: vec.x / mag, y: vec.y / mag }
}

function fabrik(bones, bone_ids, root, target) {
  let nextPos = { x: target.x, y: target.y }
  let nextLength = 0
  let boneIdslength = bone_ids.length;
  for (let b = 0; b < boneIdslength; b++) {
    let id = bone_ids[boneIdslength - 1 - b];
    let id2 = bone_ids[boneIdslength - 1 - (b + 1)];
    const length = mulv2f(normalize(subv2(nextPos, bones[id].pos)), nextLength)
    if (b != boneIdslength - 1) {
      nextLength = magnitude(subv2(bones[id].pos, bones[id2].pos))
    }
    bones[id].pos = subv2(nextPos, length)
    nextPos.x = bones[id].pos.x
    nextPos.y = bones[id].pos.y
  }

  let prevPos = { x: root.x, y: root.y }
  let prevLength = 0
  for (let b = 0; b < boneIdslength; b++) {
    let id = bone_ids[b];
    const length = mulv2f(normalize(subv2(prevPos, bones[id].pos)), prevLength)
    if (b != boneIdslength - 1) {
      prevLength = magnitude(subv2(bones[id].pos, bones[bone_ids[b + 1]].pos))
    }
    bones[id].pos = subv2(prevPos, length)
    prevPos.x = bones[id].pos.x
    prevPos.y = bones[id].pos.y
  }
}

function arcIk(bones, ikRootIds, root, target) {
  let dist = [0.];
  let maxLength = magnitude(subv2(bones[ikRootIds[ikRootIds.length - 1]].pos, root))
  let currLength = 0
  for (let rid = 0; rid < ikRootIds.length; rid++) {
    if (rid == 0) {
      continue
    }
    length = magnitude(subv2(bones[ikRootIds[rid]].pos, bones[ikRootIds[rid - 1]].pos))
    currLength += length
    dist.push(currLength / maxLength)
  }

  const base = subv2(target, root)
  const baseAngle = Math.atan2(base.y, base.x)
  const baseMag = Math.min(magnitude(base), maxLength)
  const peak = maxLength / baseMag
  const valley = baseMag / maxLength

  for (let rid = 0; rid < ikRootIds.length; rid++) {
    if (rid == 0) {
      continue
    }
    bones[ikRootIds[rid]].pos = {
      x: bones[ikRootIds[rid]].pos.x * valley,
      y: root.y + (1 - peak) * Math.sin(dist[rid] * 3.14) * baseMag
    }
    const rotated = rotate(subv2(bones[ikRootIds[rid]].pos, root), baseAngle)
    bones[ikRootIds[rid]].pos = addv2(rotated, root)
  }
}

function inverseKinematics(bones, ikRootIds) {
  let ikRots = []
  for (let i = 0; i < ikRootIds.length; i++) {
    family = bones[ikRootIds[i]]

    const root = { x: family.pos.x, y: family.pos.y };
    const target = { x: bones[family.ik_target_id].pos.x, y: bones[family.ik_target_id].pos.y };

    // run the appropriate IK mode
    if (family.ik_mode == "FABRIK") {
      for (f = 0; f < 10; f++) {
        fabrik(bones, family.ik_bone_ids, root, target)
      }
    } else {
      arcIk(bones, family.ik_bone_ids, root, target)
    }

    // the IK modes above only change bone position; now rotate the bones such that they point to the next one
    const endBone = bones[family.ik_bone_ids[family.ik_bone_ids.length - 1]]
    let tipPos = { x: endBone.pos.x, y: endBone.pos.y };
    for (let b = 0; b < family.ik_bone_ids.length; b++) {
      if (b == 0) {
        continue
      }
      let bid = family.ik_bone_ids[family.ik_bone_ids.length - 1 - b];
      const dir = subv2(tipPos, bones[bid].pos)
      bones[bid].rot = Math.atan2(dir.y, dir.x)
      tipPos = { x: bones[bid].pos.x, y: bones[bid].pos.y };
    }

    // apply constraints, if appropriate
    const jointDir = normalize(subv2(bones[family.ik_bone_ids[1]].pos, root))
    const baseDir = normalize(subv2(target, root))
    const dir = jointDir.x * baseDir.y - baseDir.x * jointDir.y;
    const baseAngle = Math.atan2(baseDir.y, baseDir.x)
    const cw = family.ik_constraint == "Clockwise" && dir > 0.;
    const ccw = family.ik_constraint == "CounterClockwise" && dir < 0.;
    if (cw || ccw) {
      for (id of family.ik_bone_ids) {
        bones[id].rot = -bones[id].rot + baseAngle * 2;
      }
    }

    /* save rots to hash */
    for (let b = 0; b < family.ik_bone_ids.length; b++) {
      if (b == family.ik_bone_ids.length - 1) {
        continue
      }
      ikRots[bones[family.ik_bone_ids[b]].id] = bones[family.ik_bone_ids[b]].rot
    }
  }

  return ikRots
}

function SkfGenericGetBoneTexture(texName, styles) {
  finalTex = false
  for (style of styles) {
    for (tex of style.textures) {
      if (texName == tex.name && !finalTex) {
        return tex;
      }
    }
  }
}

function SkfGenericAnimate(bones, anims, frames, smoothFrames) {
  for (let a = 0; a < anims.length; a++) {
    for (k = 0; k < anim.keyframes.length; k++) {
      let kf = anims[a].keyframes[k];

      // only prev keyframes are considered
      if (kf.frame > frames[a]) {
        break;
      }

      if (kf.next_kf == -1) {
        kf.next_kf = k;
      }
      let nextKf = anims[a].keyframes[kf.next_kf];

      // this is a redundant keyframe if the next one is also before this frame
      if (nextKf.frame < frames[a] && kf.next_kf != k) {
        continue;
      }

      let bone = bones[kf.bone_id];

      let c1 = kf.element[0];
      let c2 = kf.element[kf.element.length - 1];
      if (c1 == 'P' && c2 == 'X')
        bone.pos.x = interpolateKeyframes(bone.pos.x, kf, nextKf, frames[a], smoothFrames[a]);
      if (c1 == 'P' && c2 == 'Y')
        bone.pos.y = interpolateKeyframes(bone.pos.y, kf, nextKf, frames[a], smoothFrames[a]);
      if (c1 == 'R' && c2 == 'n')
        bone.rot = interpolateKeyframes(bone.rot, kf, nextKf, frames[a], smoothFrames[a]);
      if (c1 == 'S' && c2 == 'X')
        bone.scale.x = interpolateKeyframes(bone.scale.x, kf, nextKf, frames[a], smoothFrames[a]);
      if (c1 == 'S' && c2 == 'Y')
        bone.scale.y = interpolateKeyframes(bone.scale.y, kf, nextKf, frames[a], smoothFrames[a]);
      if (c1 == 'H' && c2 == 'n') {
        bone.hidden = kf.value == 1;
      }
    }
  }

  /* reset bone fields w/ bitmasks */
  const animatedMap = new Map();
  const FLAGS = {
    PositionX: 1 << 0,
    PositionY: 1 << 1,
    Rotation: 1 << 2,
    ScaleX: 1 << 3,
    ScaleY: 1 << 4,
    Hidden: 1 << 5,
  };
  for (const anim of anims) {
    for (const kf of anim.keyframes) {
      let mask = animatedMap.get(kf.bone_id) || 0;
      mask |= FLAGS[kf.element] || 0;
      animatedMap.set(kf.bone_id, mask);
    }
  }
  for (const bone of bones) {
    const mask = animatedMap.get(bone.id) || 0;
    if (!(mask & FLAGS.PositionX)) bone.pos.x = bone.init_pos.x;
    if (!(mask & FLAGS.PositionY)) bone.pos.y = bone.init_pos.y;
    if (!(mask & FLAGS.Rotation)) bone.rot = bone.init_rot;
    if (!(mask & FLAGS.ScaleX)) bone.scale.x = bone.init_scale.x;
    if (!(mask & FLAGS.ScaleY)) bone.scale.y = bone.init_scale.y;
    if (!(mask & FLAGS.Hidden)) bone.hidden = bone.init_hidden || false;
  }
}

function interpolate(current, max, startVal, endVal, startHandle, endHandle) {
  // snapping behavior for None transition preset
  if (startHandle.y == 999. && endHandle.y == 999.) {
    return startVal;
  }
  if (max == 0 || current >= max) {
    return endVal;
  }

  // solve for time (x axis) with Newton-Raphson
  let initial = current / max;
  let t = initial;
  for (let i = 0; i < 5; i++) {
    let x = cubic_bezier(t, startHandle.x, endHandle.x);
    let dx = cubic_bezier_derivative(t, startHandle.x, endHandle.x);
    if (Math.abs(dx) < 1e-5) {
      break;
    }
    t -= (x - initial) / dx;
    if (t > 1) {
      t = 1;
    } else if (t < 0) {
      t = 0
    }
  }

  let progress = cubic_bezier(t, startHandle.y, endHandle.y);
  return startVal + (endVal - startVal) * progress
}

function cubic_bezier(t, p1, p2) {
  let u = 1. - t;
  return 3. * u * u * t * p1 + 3. * u * t * t * p2 + t * t * t
}

function cubic_bezier_derivative(t, p1, p2) {
  let u = 1. - t;
  return 3. * u * u * p1 + 6. * u * t * (p2 - p1) + 3. * t * t * (1. - p2);
}

function _skfBinarySearchKeyframes(keyframes, frame) {
  let lo = 0, hi = keyframes.length - 1;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    const kf = keyframes[mid];
    if (kf.frame <= frame) {
      lo = mid + 1;
    } else {
      hi = mid - 1;
    }
  }
  return lo
}

function interpolateKeyframes(field, prevKf, nextKf, frame, smoothFrame) {
  const totalFrames = nextKf.frame - prevKf.frame
  const currentFrame = frame - prevKf.frame
  const result = interpolate(currentFrame, totalFrames, prevKf.value, nextKf.value, nextKf.start_handle, nextKf.end_handle)
  return interpolate(currentFrame, smoothFrame, field, result, { x: 0, y: 0 }, { x: 0, y: 0 })
}

function resetInheritance(cachedBones, ogBones) {
  for (let b = 0; b < cachedBones.length; b++) {
    cachedBones[b].pos = ogBones[b].pos;
    cachedBones[b].scale = ogBones[b].scale;
    cachedBones[b].rot = ogBones[b].rot;
    cachedBones[b].hidden = ogBones[b].hidden;
  }
}

function inheritance(bones, ikRots, armature_bones) {
  for (let b = 0; b < bones.length; b++) {
    if (bones[b].parent_id == -1) {
      continue;
    }
    const parent = bones[bones[b].parent_id]

    let orbit_rot = parent.rot;
    // apply orbital difference, if rotation resistance physics is active
    if (armature_bones && armature_bones[b].phys_sway > 0) {
      orbit_rot -= armature_bones[b].phys_global_orbit_diff;
    }

    bones[b].rot += orbit_rot

    bones[b].scale = mulv2(bones[b].scale, parent.scale)
    bones[b].pos = mulv2(bones[b].pos, parent.scale)
    /* rotate child around parent as if it were orbitting */
    bones[b].pos = rotate(bones[b].pos, orbit_rot)
    bones[b].pos = addv2(bones[b].pos, parent.pos)

    if (ikRots[bones[b].id]) {
      bones[b].rot = ikRots[bones[b].id]
    }

    if (armature_bones) {
      if (armature_bones[b].phys_rot_damping > 0) {
        bones[b].rot = armature_bones[b].phys_global_rot;
      }
      if (armature_bones[b].phys_pos_damping > 0) {
        bones[b].pos = armature_bones[b].phys_global_pos;
      }
      if (armature_bones[b].phys_scale_damping > 0) {
        bones[b].scale = armature_bones[b].phys_global_scale;
      }
    }
  }

  return bones
}

function shortest_angle_delta(from, to) {
  let pi = 3.141592653589793;
  let tau = pi * 2.0;
  let delta = to - from
  while (delta > pi) {
    delta -= tau;
  }
  while (delta < -pi) {
    delta += tau;
  }
  return delta
}

function simulate_physics(armature_bones, constructed_bones) {
  for (let b = 0; b < armature_bones.length; b++) {
    s = { x: 0.3, y: 0.3 }
    e = { x: 0.6, y: 0.6 }
    arm_bone = armature_bones[b]
    const_bone = constructed_bones[b]
    if (!arm_bone.phys_global_pos) {
      arm_bone.phys_global_pos = { x: 0, y: 0 }
    }
    if (!arm_bone.phys_global_orbit) {
      arm_bone.phys_global_orbit = 0
      arm_bone.phys_global_orbit_vel = 0
    }
    prev_pos = { x: arm_bone.phys_global_pos.x, y: arm_bone.phys_global_pos.y }

    // interpolate position
    if (arm_bone.phys_pos_damping || arm_bone.phys_sway) {
      phys_pos = arm_bone.phys_global_pos
      if (!arm_bone.phys_pos_damping) {
        arm_bone.phys_pos_damping = 0
      }
      damping = { x: arm_bone.phys_pos_damping, y: arm_bone.phys_pos_damping }

      // ratio
      if (arm_bone.phys_pos_rato) {
        if (arm_bone.phys_pos_ratio < 0.0) {
          damping.y *= 1.0 - Math.abs(arm_bone.phys_pos_ratio)
        } else if (arm_bone.phys_pos_ratio > 0.0) {
          damping.x *= 1.0 - arm_bone.phys_pos_ratio
        }
      }

      arm_bone.phys_global_pos = {
        x: interpolate(2.0, damping.x, phys_pos.x, const_bone.pos.x, s, e),
        y: interpolate(2.0, damping.y, phys_pos.y, const_bone.pos.y, s, e),
      }
    }

    // interpolate scale
    if (arm_bone.phys_scale_damping) {
      phys_scale = arm_bone.phys_global_scale
      damping = { x: arm_bone.phys_scale_damping, y: arm_bone.phys_scale_damping }

      // ratio
      if (arm_bone.phys_scale_ratio) {
        damping.y *= 1.0 - Math.abs(arm_bone.phys_scale_ratio)
      }
      else if (arm_bone.phys_scale_ratio) {
        damping.x *= 1.0 - arm_bone.phys_scale_ratio
      }

      phys_scale.x = interpolate(2.0, damping.x, phys_scale.x, const_bone.scale.x, s, e)
      phys_scale.y = interpolate(2.0, damping.y, phys_scale.y, const_bone.scale.y, s, e)
    }

    // interpolate rotation
    if (arm_bone.phys_rot_damping) {
      rot = shortest_angle_delta(arm_bone.phys_global_rot, const_bone.rot)
      arm_bone.phys_global_rot += rot / arm_bone.phys_rot_damping
    }

    parent = constructed_bones.find((b) => b.id == const_bone.parent_id);
    if (arm_bone.phys_sway && parent) {
      // interpolate to the angle difference between bone and parent
      diff = normalize(subv2(const_bone.pos, parent.pos))
      diff_angle = Math.atan2(diff.y, diff.x)
      rest_rot = shortest_angle_delta(arm_bone.phys_global_orbit, diff_angle)

      // apply bounce
      if (arm_bone.phys_rot_bounce && arm_bone.phys_rot_bounce <= 1) {
        bounce = arm_bone.phys_rot_bounce
        rest_rot += arm_bone.phys_global_orbit_vel / (2.0 - bounce)
        arm_bone.phys_global_orbit_vel = rest_rot
      }
      arm_bone.phys_global_orbit += rest_rot / 10.0

      // swing orbit based on position momentum
      vel = normalize(subv2(arm_bone.phys_global_pos, prev_pos))
      angle = Math.atan2(-vel.y, -vel.x)
      vel_rot = shortest_angle_delta(arm_bone.phys_global_orbit, angle)
      strength = magnitude(subv2(arm_bone.phys_global_pos, prev_pos)) / 1000
      arm_bone.phys_global_orbit += vel_rot * strength * arm_bone.phys_sway

      // apply difference in final angle and orbit
      arm_bone.phys_global_orbit_diff = diff_angle - arm_bone.phys_global_orbit
    }
  }
}


function SkfGenericConstruct(rawBones, ikRootIds, cachedBones) {
  if (!cachedBones) {
    cachedBones = structuredClone(rawBones);
  } else {
    cachedBones.sort((a, b) => (a.id > b.id) ? 1 : -1)
  }

  resetInheritance(cachedBones, rawBones);
  inheritance(cachedBones, [])

  let ikRots = {}
  if (ikRootIds) {
    ikRots = inverseKinematics(cachedBones, ikRootIds)
    resetInheritance(cachedBones, rawBones);
    inheritance(cachedBones, ikRots)
  }

  simulate_physics(rawBones, cachedBones)
  resetInheritance(cachedBones, rawBones);
  inheritance(cachedBones, ikRots, rawBones)

  constructVerts(cachedBones)

  return cachedBones;
}

function constructVerts(bones) {
  for (let b = 0; b < bones.length; b++) {
    if (!bones[b].vertices) {
      continue;
    }

    for (let v = 0; v < bones[b].vertices.length; v++) {
      bones[b].vertices[v].pos = bones[b].vertices[v].init_pos;
      bones[b].vertices[v].pos = inheritVert(bones[b].vertices[v].pos, bones[b]);
    }

    for (let bi = 0; bi < bones[b].binds.length; bi++) {
      if (bones[b].binds[bi].bone_id == -1) {
        continue;
      }

      const bindBone = bones[bones[b].binds[bi].bone_id];

      for (bind_vert of bones[b].binds[bi].verts) {
        if (!bones[b].binds[bi].is_path) {
          let vert = bones[b].vertices[bind_vert.id];
          endPos = subv2(inheritVert(vert.init_pos, bindBone), vert.pos);
          vert.pos = addv2(vert.pos, mulv2f(endPos, bind_vert.weight));
          continue;
        }

        const prev = bi > 0 ? bi - 1 : bi
        const next = bi + 1 <= bones[b].binds.length - 1 ? bi + 1 : bones[b].binds.length - 1
        const bone = bones[b];
        const prevBone = bones[bone.binds[prev].bone_id];
        const nextBone = bones[bone.binds[next].bone_id];

        const prevDir = subv2(bindBone.pos, prevBone.pos)
        const nextDir = subv2(nextBone.pos, bindBone.pos)
        const prevNorm = normalize({ x: -prevDir.y, y: prevDir.x })
        const nextNorm = normalize({ x: -nextDir.y, y: nextDir.x })
        const average = addv2(prevNorm, nextNorm);
        const normAngle = Math.atan2(average.y, average.x)

        let vert = bones[b].vertices[bind_vert.id]
        vert.pos = addv2(vert.init_pos, bindBone.pos)
        let rotated = rotate(subv2(vert.pos, bindBone.pos), normAngle)
        vert.pos = addv2(bindBone.pos, mulv2f(rotated, bind_vert.weight))
      }
    }
  }
}

function inheritVert(pos, bone) {
  pos = mulv2(pos, bone.scale);
  pos = rotate(pos, bone.rot);
  pos = addv2(pos, bone.pos);
  return pos
}
