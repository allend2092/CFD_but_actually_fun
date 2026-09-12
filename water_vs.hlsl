// water_vs.hlsl
// Vertex shaders for water, crate, and jetski+rider.
//
// TRANSFORM CONTRACT (who owns the transform):
//   main       : water vertices arrive in WORLD space (CPU-displaced) -> view*proj only.
//   mainModel  : crate vertices are LOCAL (origin-centered) -> needs the model matrix.
//   mainJetski : rig_pose() emits WORLD-space vertices (hull transform already baked
//                in) -> view*proj only. Applying the model matrix here a second time
//                double-transforms the rider (the old "shrinking rider" bug).
cbuffer RootConstants : register(b0)
{
    row_major float4x4 mvpWater;   // view * proj
    row_major float4x4 mvpModel;   // model * view * proj (crate only)
    row_major float4x4 model;      // crate model matrix
    float4 lightDir;
    float4 camPosTime;
};

struct VSIn
{
    float3 position : POSITION;
    float3 normal : NORMAL;
};

struct VSOut
{
    float4 clip : SV_Position;
    float3 world : TEXCOORD0;
    float3 normal : TEXCOORD1;
};

VSOut main(VSIn input)
{
    VSOut o;
    o.world = input.position;
    o.normal = input.normal;
    o.clip = mul(float4(input.position, 1.0), mvpWater);
    return o;
}

VSOut mainModel(VSIn input)
{
    VSOut o;
    o.world = mul(float4(input.position, 1.0), model).xyz;
    o.normal = normalize(mul(input.normal, (float3x3)model));
    o.clip = mul(float4(input.position, 1.0), mvpModel);
    return o;
}

struct VSInJetski
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float3 color : COLOR;
};

struct VSOutJetski
{
    float4 clip : SV_Position;
    float3 world : TEXCOORD0;
    float3 normal : TEXCOORD1;
    float3 color : TEXCOORD2;
};

VSOutJetski mainJetski(VSInJetski input)
{
    VSOutJetski o;
    o.world = input.position;   // already world space from rig_pose
    o.normal = input.normal;
    o.color = input.color;
    o.clip = mul(float4(input.position, 1.0), mvpWater);
    return o;
}