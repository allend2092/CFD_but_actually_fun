# CFD but actually fun

A real-time, **asset-free** water-and-boat scene rendered in **Direct3D 12**. The ocean, the
sky, the light, and the wake are all **procedural** — computed in-shader from a handful of
constants — with no textures, no skybox, and no baked assets. A planing rigid-body boat with a
rider rides the Gerstner waves under a physically-motivated sky that moves with the time of day.

> **Status: work in progress.** The core ocean, boat, rider, and sky pipeline render end-to-end.
> See [Roadmap](#roadmap) for what's still coming.

## The scene

| | |
|---|---|
| ![Caribbean Evening — the default sunset vista](images/sunset.png) | **Caribbean Evening** — the default vista: a low western sun, warm horizon, and a
deep blue ocean. This is what you see when you launch the app. |

| | |
|---|---|
| ![A cloudy sky](images/cloudy.png) | **Cloudy** — overcast cloud cover and a muted, hazy grade. |
| ![Dusk — time scrubbed toward evening](images/dusk.png) | **Dusk** — the sun scrubbed toward the horizon with `[` / `]`. |
| ![The boat at night](images/night_boat.png) | **Night** — the boat under a near-black sky with a faint, dim horizon. |

## Controls

| Key | Action |
|---|---|
| `W` / `S` | Throttle up / down (with coast-down decay) |
| `A` / `D` | Steer left / right (with return-to-center decay) |
| `R` | Reset the boat to the center of the water patch |
| `F1` | Toggle **wireframe** rendering of the water |
| `P` | Toggle the **rider** between the static pose and the full IK rig |
| `[` / `]` | Scrub the **time of day** (the sun moves along its arc, ~1 h/s) |
| `T` | Jump to the next **sky vista** (cycles all seven presets) |

### Wireframe

`F1` swaps the solid water shader for a wireframe one, exposing the simulation grid.

| | |
|---|---|
| ![Wireframe, day](images/wireframe_day.png) | Wireframe in **daylight** — the Gerstner grid under a bright sky. |
| ![Wireframe, dusk](images/wireframe_dusk.png) | Wireframe at **dusk** — the same grid lit by the low sun. |

## What's procedural

- **Ocean** — summed Gerstner waves give the surface its shape and normals; a boat-locked
  **wake heightfield** adds the V-shaped disturbance the hull leaves behind. Foam is driven by the
  boat's state and the wake in the water shader.
- **Boat** — a 2-DOF **rigid body** that planes: it takes on speed, trim, and roll as you
  throttle, and settles back to calm when you let off.
- **Rider** — either a **static pose** rigid on the hull (default) or a full **inverse-kinematics
  rig** (`P`) that tracks the hull's motion.
- **Sky** — a fullscreen pass that reconstructs the world-space view ray per pixel and shades it
  with:
  - a vertical day↔night **gradient**,
  - **Mie** forward-scatter (horizon haze + sun glow),
  - **Rayleigh**-motivated scattering (blue sky, reddening toward the sun),
  - a bright **sun disc**,
  - **FBM procedural clouds** with coverage, drift, and feature scale,
  - a per-vista **color grade** (palette).
- **Light** — one sun, driven by the time of day (sunrise 06:00 east, noon 12:00 due south at
  75°, sunset 18:00 west), lights the sky, the water, the crate buoy, and the jet-ski identically.
- **Clear color** — the CPU computes the same horizon color the shader would, so any uncovered
  pixel melts into the sky instead of showing a seam.

The render target is a plain `R8G8B8A8_UNORM` swap chain — no sRGB view, no bloom, no tonemapping.
Values above 1.0 clamp to white.

## Sky vistas

Press `T` to cycle, or `[` / `]` to scrub the time of day continuously. Seven presets ship:

| # | Vista | Default time | Character |
|---|---|---|---|
| 0 | Caribbean Evening | 17:36 | warm low sun, pink-orange horizon, deep blue water |
| 1 | Caribbean Noon | 12:00 | bright blue, high sun, crisp |
| 2 | Caribbean Dawn | 06:24 | soft rose horizon, gentle light |
| 3 | Hazy Tropical | 15:00 | heavy white haze, washed-out warmth |
| 4 | Clear Blue | 13:00 | saturated blue, almost no clouds |
| 5 | Overcast | 14:00 | flat grey, dense cloud cover, muted grade |
| 6 | Night | 23:30 | near-black sky, faint horizon, dim light |

## Building

- **Toolchain:** Visual Studio 2022, MSVC `v143`, **C++20**, Windows 10 SDK.
- **Project:** `CFD_but_actually_fun.vcxproj` (a Windows console app — a console window stays
  open showing live telemetry).
- **GPU:** any D3D12-capable adapter; the first available is used.

```
1. Open CFD_but_actually_fun.sln in Visual Studio 2022.
2. Build (x64).
3. Run from the project directory so D3DCompileFromFile() can find the .hlsl files,
   e.g.  D:\written_software\CFD_local_qwen_sky\
```

> **Note:** the shaders are compiled at runtime with `D3DCompileFromFile`, so the `.hlsl` files
> must sit next to the exe (or be found relative to the working directory). Build and run from the
> source folder and it just works.

The D3D12 debug layer is enabled at startup; run under the debugger to see validation messages in
the VS **Output** window.

## Project layout

```
CFD_but_actually_fun.sln
CFD_but_actually_fun.vcxproj
main.cpp                  D3D12 setup, game loop, boat/camera, sky packing, rendering
sky.hlsli                 shared sky model (SkyRoot, SkyViewDir, SkyColor, SkyClouds)
water_vs.hlsl             water + model + jet-ski + sky vertex shaders
water_ps.hlsl             water + crate + jet-ski + sky pixel shaders
images/                   screenshots referenced by this README
assets/
  boat_sim.h              2-DOF planing rigid-body boat (trim + roll)
  wake_field.h            boat-locked interactive wake heightfield
  jetski_asset.h/.cpp     jet-ski hull/rider mesh data
  jetski_internal.h       rig definition (complete type)
  jetski_ik.cpp           rider inverse-kinematics solver
  *.obj                   jet-ski pose exports (idle / bounce / turn)
```

## Telemetry

The console prints a live line each frame:

```
[telemetry] speed 9.4 m/s  hdg +11.7 deg  pos (10.4, 56.9)  trim +0.0 deg  roll +0.2 deg  rider IK  sky Caribbean Evening 17:36
```

`rider` shows `STATIC` or `IK` depending on the `P` toggle; `sky` shows the active vista and the
current time of day.

## Roadmap

Work in progress — likely next:

- [ ] Rider IK hardening around the ~180° yaw case (the rig degenerates at extreme heading).
- [ ] LDR→HDR upgrade: sRGB render target, exposure + tonemap for the sun and highlights.
- [ ] Water shading polish: specular sun glint, depth-based color, foam detail.
- [ ] Camera work: chase-cam framing, FOV/speed feel.
- [ ] Performance: larger water grid, more wave components, cloud octaves.
- [ ] Packaging: hide the console, a proper window, settings.

## Tech stack

Direct3D 12 (flip-discard swap chain, `R8G8B8A8_UNORM`), HLSL `vs_5_0`/`ps_5_0` compiled at
runtime via `D3DCompiler`, C++20, MSVC. All geometry is procedural or generated at startup — the
only non-generated assets are the small jet-ski mesh exports in `assets/`.
