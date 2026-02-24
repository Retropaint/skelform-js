function SkfGenericFormatFrame(frame, anim, isReverse, isLoop) {
  const lastFrame = anim.keyframes[anim.keyframes.length - 1].frame
  if (isLoop) {
    frame %= lastFrame + 1
  }

  if (isReverse) {
    frame = lastFrame - frame
  }

  return frame
}

function SkfGenericTimeFrame(time, anim, isReverse, isLoop) {
  const elapsed = time / 1000
  const frametime = 1 / anim.fps
  const frame = elapsed / frametime
  return SkfGenericFormatFrame(frame, anim, isReverse, isLoop)
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
  let nextPos = target
  let nextLength = 0
  let rev_bone_ids = structuredClone(bone_ids)
  rev_bone_ids.reverse().forEach((id, b) => {
    const length = mulv2f(normalize(subv2(nextPos, bones[id].pos)), nextLength)
    if (b != rev_bone_ids.length - 1) {
      nextLength = magnitude(subv2(bones[id].pos, bones[rev_bone_ids[b + 1]].pos))
    }
    bones[id].pos = subv2(nextPos, length)
    nextPos = structuredClone(bones[id].pos)
  })

  let prevPos = root
  let prevLength = 0
  bone_ids.forEach((id, b) => {
    const length = mulv2f(normalize(subv2(prevPos, bones[id].pos)), prevLength)
    if (b != rev_bone_ids.length - 1) {
      prevLength = magnitude(subv2(bones[id].pos, bones[bone_ids[b + 1]].pos))
    }
    bones[id].pos = subv2(prevPos, length)
    prevPos = structuredClone(bones[id].pos)
  })
}

function arcIk(bones, ikRootIds, root, target) {
  let dist = [0.];
  let maxLength = magnitude(subv2(bones[ikRootIds[ikRootIds.length - 1]].pos, root))
  let currLength = 0
  ikRootIds.forEach((rootId, rid) => {
    if (rid == 0) { return }
    length = magnitude(subv2(bones[rootId].pos, bones[ikRootIds[rid - 1]].pos))
    currLength += length
    dist.push(currLength / maxLength)
  })

  const base = subv2(target, root)
  const baseAngle = Math.atan2(base.y, base.x)
  const baseMag = Math.min(magnitude(base), maxLength)
  const peak = maxLength / baseMag
  const valley = baseMag / maxLength

  ikRootIds.forEach((rootId, rid) => {
    if (rid == 0) { return }
    bones[rootId].pos = {
      x: bones[rootId].pos.x * valley,
      y: root.y + (1 - peak) * Math.sin(dist[rid] * 3.14) * baseMag
    }
    const rotated = rotate(subv2(bones[rootId].pos, root), baseAngle)
    bones[rootId].pos = addv2(rotated, root)
  })
}

function inverseKinematics(bones, ikRootIds) {
  let ikRots = []
  ikRootIds.forEach(rootId => {
    family = bones[rootId]
    const bone_ids = structuredClone(family.ik_bone_ids);

    const root = structuredClone(family.pos);
    const target = structuredClone(bones[family.ik_target_id].pos);
    if (family.ik_mode == "FABRIK") {
      for (i = 0; i < 10; i++) {
        fabrik(bones, structuredClone(bone_ids), root, target)
      }
    } else {
      arcIk(bones, structuredClone(bone_ids), root, target)
    }



    const endBone = bones[bone_ids[bone_ids.length - 1]]
    let tipPos = structuredClone(endBone.pos);
    let rev_bone_ids = structuredClone(bone_ids)
    rev_bone_ids.reverse().forEach((bid, b) => {
      if (b == 0) {
        return
      }
      const dir = subv2(tipPos, bones[bid].pos)
      bones[bid].rot = Math.atan2(dir.y, dir.x)
      tipPos = structuredClone(bones[bid].pos)
    })

    const jointDir = normalize(subv2(bones[bone_ids[1]].pos, root))
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
    bone_ids.forEach((bid, b) => {
      if (b == bone_ids.length - 1) {
        return
      }
      ikRots[bones[bid].id] = bones[bid].rot
    })
  })

  return ikRots
}

function SkfGenericGetBoneTexture(texName, styles) {
  finalTex = false
  styles.forEach(style => {
    style.textures.forEach(tex => {
      if (texName == tex.name && !finalTex) {
        finalTex = tex
      }
    })
  })
  return finalTex
}

function SkfGenericAnimate(bones, anims, frames, smoothFrames) {
  anims.forEach((anim, a) => {
    bones.forEach(bone => {
      bone.pos.x = interpolateKeyframes(bone.id, bone.pos.x, anim.keyframes, "PositionX", frames[a], smoothFrames[a])
      bone.pos.y = interpolateKeyframes(bone.id, bone.pos.y, anim.keyframes, "PositionY", frames[a], smoothFrames[a])
      bone.rot = interpolateKeyframes(bone.id, bone.rot, anim.keyframes, "Rotation", frames[a], smoothFrames[a])
      bone.scale.x = interpolateKeyframes(bone.id, bone.scale.x, anim.keyframes, "ScaleX", frames[a], smoothFrames[a])
      bone.scale.y = interpolateKeyframes(bone.id, bone.scale.y, anim.keyframes, "ScaleY", frames[a], smoothFrames[a])
    })
  })

  bones.forEach(bone => {
    if (!isAnimated(bone.id, anims, "PositionX")) {
      bone.pos.x = interpolate(frames[0], smoothFrames[0], bone.pos.x, bone.init_pos.x);
    }
    if (!isAnimated(bone.id, anims, "PositionY")) {
      bone.pos.y = interpolate(frames[0], smoothFrames[0], bone.pos.y, bone.init_pos.y);
    }
    if (!isAnimated(bone.id, anims, "Rotation")) {
      bone.rot = interpolate(frames[0], smoothFrames[0], bone.rot, bone.init_rot);
    }
    if (!isAnimated(bone.id, anims, "ScaleX")) {
      bone.scale.x = interpolate(frames[0], smoothFrames[0], bone.scale.x, bone.init_scale.x);
    }
    if (!isAnimated(bone.id, anims, "ScaleY")) {
      bone.scale.y = interpolate(frames[0], smoothFrames[0], bone.scale.y, bone.init_scale.y);
    }
  })
}

function isAnimated(bone_id, anims, element) {
  let yes = false;
  anims.forEach((anim, a) => {
    anim.keyframes.forEach(kf => {
      if (kf.bone_id == bone_id && kf.element == element) {
        yes = true;
      }
    })
  })
  return yes
}

function interpolate(current, max, startVal, endVal) {
  if (max == 0 || current >= max) {
    return endVal
  }
  const interp = current / max
  const end = endVal - startVal
  const result = startVal + (end * interp)

  return result
}

function interpolateKeyframes(bone_id, field, keyframes, element, frame, smoothFrames) {
  let prev = false;
  let next = false;
  for (kf of keyframes) {
    if (kf.frame <= frame && kf.element == element && kf.bone_id == bone_id) {
      prev = kf
    }
  }

  for (kf of keyframes) {
    if (kf.frame > frame && kf.element == element && kf.bone_id == bone_id) {
      next = kf
      break
    }
  }

  if (!prev) {
    prev = next
  }
  if (!next) {
    next = prev
  }

  if (!prev && !next) {
    return field;
  }

  const totalFrames = next.frame - prev.frame
  const currentFrame = frame - prev.frame

  const result = interpolate(currentFrame, totalFrames, prev.value, next.value)
  return interpolate(currentFrame, smoothFrames, field, result)
}

function inheritance(bones, ikRots) {
  bones.forEach((bone, b) => {
    if (bone.parent_id == -1) {
      return;
    }
    const parent = bones[bone.parent_id]

    bones[b].rot += parent.rot
    bones[b].scale = mulv2(bones[b].scale, parent.scale)
    bones[b].pos = mulv2(bones[b].pos, parent.scale)
    /* rotate child around parent as if it were orbitting */
    bones[b].pos = rotate(bones[b].pos, parent.rot)
    bones[b].pos = addv2(bones[b].pos, parent.pos)

    if (ikRots[bone.id]) {
      bones[b].rot = ikRots[bone.id]
    }
  })

  return bones
}

function SkfGenericConstruct(rawBones, ikRootIds) {
  const inhBones = inheritance(structuredClone(rawBones), [])
  const ikRots = inverseKinematics(structuredClone(inhBones), ikRootIds)
  let finalBones = inheritance(structuredClone(rawBones), ikRots)
  constructVerts(finalBones)
  return finalBones
}

function constructVerts(bones) {
  bones.forEach((_, b) => {
    if (!bones[b].vertices) {
      return
    }

    for (vert of bones[b].vertices) {
      vert.pos = inheritVert(vert.init_pos, bones[b])
    }

    bones[b].binds.forEach((bind, bi) => {
      if (bind.bone_id == -1) {
        return;
      }

      const bindBone = bones.find((b) => b.id == bind.bone_id);

      for (bind_vert of bones[b].binds[bi].verts) {
        if (!bind.is_path) { continue }

        const prev = bi > 0 ? bi - 1 : bi
        const next = bi + 1 <= bones[b].binds.length - 1 ? bi + 1 : bones[b].binds.length - 1
        const bone = bones[b];
        const prevBone = bones.find((b) => b.id == bone.binds[prev].bone_id)
        const nextBone = bones.find((b) => b.id == bone.binds[next].bone_id)

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
    })
  })
}

function inheritVert(pos, bone) {
  pos = mulv2(pos, bone.scale);
  pos = rotate(pos, bone.rot);
  pos = addv2(pos, bone.pos);
  return pos
}
