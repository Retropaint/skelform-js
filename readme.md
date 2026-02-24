# SkelForm-js

Generic SkelForm runtime for Javascript.

## Installation

The `runtime.js` is designed to be added via a `<script>` tag, either by
downloading the file and fetching locally (recommended) or from
`https://skelform.org/runtime.js`.

## Using .skf Files

This generic runtime does not provide a way to extract `.skf` files.

Check out the [Web Player](https://github.com/Retropaint/skelform-web-player)
for an example (utilizes [JSZip](https://stuk.github.io/jszip/)).

## Usage

For clarity's sake, Typescript types will be used for function parameters.

```typescript
SkfAnimateGeneric(bones: Bone[], anims: Animation[], frames: number[], smoothFrames: number[])
```

Animates the provided bones with the animations.

```typescript
SkfConstructGeneric(bones: Bone[], ikRootIds: number[]): bones
```

Returns a constructed version of the provided bones.

```typescript
SkfFormatFrameGeneric(frame: number, anim: number, isReverse: bool, isLoop: bool): number
```

Returns the proper, formatted frame based on the animation.

```typescript
SkfTimeFrameGeneric(time: number, anim: number, isReverse: bool, isLoop: bool): number
```

Returns the frame based on the provided time (in seconds).
