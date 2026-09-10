cbuffer RootConstants : register(b0)
{
    row_major float4x4 mvp;
    float4 lightDir;     // xyz = direction from surface toward the light
    float4 camPosTime;   // xyz = camera world position, w = time in seconds
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
    o.clip = mul(float4(input.position, 1.0), mvp);
    return o;
}