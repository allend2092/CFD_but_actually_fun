// water_ps_wake.hlsl
// Drop-in replacement for water_ps.hlsl in CFD_but_actually_fun.
// Adds the interactive-wake FOAM (white) on top of the existing water shading.
//
// ARCHITECTURE (matches the repo's CPU-displaced water):
//   The water SURFACE displacement (Gerstner + interactive wake field) is done on
//   the CPU in UpdateWater() -> the grid arrives pre-displaced, so the VS is a flat
//   pass-through (unchanged). The wake field is what makes the surface heave in the
//   real V + let the boat ride its own wake. This file adds only the VISUAL foam:
//   a procedural Kelvin V + bow spray, oriented by the boat's forward vector that
//   main.cpp puts in the cbuffer. No texture/SRV/sampler needed -> the ONLY D3D12
//   change is the push-constant cbuffer growing 56 -> 64 floats.
//
// Best-effort: NOT compile-verified on the RHEL dev host (no fxc/D3D). Expect a
// trivial typo; the C++ physics/wake side is Linux-verified.
//
// cbuffer: the original 56 floats are UNCHANGED at the front, so water_vs.hlsl
// (which declares the shorter cbuffer) still works. boatState + wakeParams are
// appended (floats 56..63) -> 64 total, exactly the D3D12 256-byte push-constant limit.
//
// The procedural sky lives in sky.hlsli (shared with water_vs.hlsl). Its ray
// reconstruction + style parameters are in a SECOND root constant, register b1
// (SkyRoot), so they do not consume the 64-float b0 budget. The water shader
// reads b1 too, to mirror the sky in the surface (fresnel) and fog the far water
// into the sky horizon color.

#include "sky.hlsli"

cbuffer RootConstants : register(b0)
{
    row_major float4x4 mvpWater;    // 16
    row_major float4x4 mvpModel;    // 16
    row_major float4x4 model;       // 16
    float4 lightDir;                // 4   (.w unused)
    float4 camPosTime;              // 4   (.w = time)
    float4 boatState;               // 4   .x .z = boat world XZ, .y = fwdX, .w = fwdZ (unit hull forward)
    float4 wakeParams;              // 4   .x = speed (m/s), .y = throttle, .z = foamGain, .w = time
};

struct VSOut
{
    float4 clip   : SV_Position;
    float3 world  : TEXCOORD0;
    float3 normal : TEXCOORD1;
};

// Procedural Kelvin V + bow spray foam, in the boat's frame of reference.
// The V opens BEHIND the boat (opposite its forward) and decays with distance;
// the bow spray is a small bright patch in front of the hull while planing.
float3 WakeFoam(float3 world)
{
   float2 d        = world.xz - boatState.xz;
    float2 hullFwd  = normalize(float2(boatState.y, boatState.w) + 1e-4);
    float  dist     = length(d);
    float2 dir      = d * (1.0 / max(dist, 1e-4));

    // Kelvin V: half-angle ~ asin(1/pi) ~= 19.5 deg.  1 inside the cone, 0 outside.
    float  cosAng   = dot(dir, -hullFwd);
    float  inV      = smoothstep(0.88, 0.95, cosAng);        // cos(19.5deg) ~= 0.943
    float  vFade    = exp(-dist * 0.12);                      // wake decays with distance

    // Bow spray: bright, close, in front of the hull while planing.
    float  bowFwd   = dot(dir, hullFwd);
    float  bowSpray = smoothstep(0.80, 0.98, bowFwd) * (1.0 - smoothstep(0.5, 1.3, dist));

    float  moving   = smoothstep(1.5, 4.5, wakeParams.x);    // only when actually moving
    float  amt      = saturate((inV * vFade * 0.9 + bowSpray * 0.8) * moving * wakeParams.z);
    return float3(0.90, 0.95, 0.99) * amt;
}

float4 main(VSOut input) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 L = normalize(lightDir.xyz);                 // = the sky's sun (time-of-day)
    float3 V = normalize(camPosTime.xyz - input.world);
    float3 H = normalize(L + V);

    float ndl = saturate(dot(N, L));

    // Day-ness + sunset amount (matches the sky) for warm tinting + glint falloff.
    float3 sunDir  = normalize(sun.xyz);
    float  dayF    = smoothstep(-0.05, 0.15, sunDir.y);
    float  sunsetT = (1.0 - saturate(sunDir.y / 0.30)) * dayF;

    // ---- water body: what you see looking DOWN through the surface ----
    float3 deep    = lerp(float3(0.010, 0.100, 0.200), float3(0.050, 0.050, 0.120), sunsetT);
    float3 shallow = lerp(float3(0.040, 0.360, 0.440), float3(0.100, 0.180, 0.220), sunsetT);
    float3 base    = lerp(deep, shallow, ndl);
    // Low-sun raking light warms the surface (matches the sky's warmth).
    base += float3(1.0, 0.45, 0.20) * pow(ndl, 2.0) * sunsetT * 0.25;

    // ---- Schlick fresnel: more sky reflection at grazing angles ----
    float  cosTheta = saturate(dot(N, V));
    float  F = 0.02 + 0.98 * pow(1.0 - cosTheta, 5.0);

    // ---- mirror the procedural sky in the surface (clouds + sun + gradient) ----
    float3 refl = SkyColor(reflect(-V, N));

    // ---- sun glint (tight sparkle), dimmed at night ----
    float  spec = pow(saturate(dot(N, H)), 120.0) * (0.8 + 1.2 * sunsetT) * (0.3 + 0.7 * dayF);

    float  foam = smoothstep(0.30, 0.48, input.world.y);

    // ---- combine: body <-> sky by fresnel, plus glint ----
    float3 col = lerp(base * (0.35 + 0.65 * ndl), refl, F);
    col += spec;
    col = lerp(col, float3(0.85, 0.92, 0.97), foam * 0.65);

    // ---- interactive wake foam (Kelvin V + bow spray) ----
    float3 wf = WakeFoam(input.world);
    col = col + wf;
    col = lerp(col, col * 0.80 + wf, wf.r * 0.4);

    // ---- distance fog: fade the far water into the SKY horizon (seamless) ----
    float  fog = smoothstep(24.0, 44.0, length(input.world.xz - camPosTime.xz));
    float2 dxz = input.world.xz - camPosTime.xz;
    float  dl  = length(dxz);
    float2 toPatch = (dl > 1e-3) ? (dxz / dl) : float2(0.0, 1.0);
    float3 horizonSky = SkyColor(float3(toPatch.x, 0.0, toPatch.y));
    col = lerp(col, horizonSky, fog);
    return float4(col, 1.0);
}

// ---- crate + jetski shaders: unchanged from water_ps.hlsl ----------------
float4 psCrate(VSOut input) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 L = normalize(lightDir.xyz);
    float ndl = saturate(dot(N, L));

    float3 albedo = float3(0.45, 0.30, 0.16);
    float3 col = albedo * (0.25 + 0.75 * ndl);

    float3 V = normalize(camPosTime.xyz - input.world);
    float3 H = normalize(L + V);
    col += pow(saturate(dot(N, H)), 40.0) * 0.15;

    float wet = smoothstep(0.15, -0.05, input.world.y);
    col = lerp(col, col * 0.35, wet);
    return float4(col, 1.0);
}

struct VSOutJetski
{
    float4 clip   : SV_Position;
    float3 world  : TEXCOORD0;
    float3 normal : TEXCOORD1;
    float3 color  : TEXCOORD2;
};

float4 psJetski(VSOutJetski input) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 L = normalize(lightDir.xyz);
    float ndl = saturate(dot(N, L));

    float3 albedo = input.color;
    float3 col = albedo * (0.25 + 0.75 * ndl);

    float3 V = normalize(camPosTime.xyz - input.world);
    float3 H = normalize(L + V);
    col += pow(saturate(dot(N, H)), 40.0) * 0.15;

    float wet = smoothstep(0.15, -0.05, input.world.y);
    col = lerp(col, col * 0.35, wet);
    return float4(col, 1.0);
}

// ---- procedural sky: fullscreen triangle, drawn first (depth-write off) ----
// Rebuild the world-space view ray from NDC + the camera axes (SkyRoot b1) and
// shade it with the shared sky model. The sky sits at the far plane (z=1) and is
// drawn before the water, so the water overdraws the below-horizon part.
float4 skyPS(SkyVSOut input) : SV_Target
{
    float3 dir = SkyViewDir(input.ndc);
    float3 col = SkyColor(dir);
    return float4(col, 1.0);
}
