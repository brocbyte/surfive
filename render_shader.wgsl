struct VertexOutput {
    @builtin(position) position : vec4f,
    @location(0) @interpolate(flat) color : vec4f,
    @location(1) texelCoords: vec2f
}

@group(0) @binding(0) var<storage> diskOffsets : array<vec2f>;
@group(0) @binding(1) var<storage> colors : array<vec4f>;
@group(0) @binding(2) var input_texture: texture_2d<f32>;
@vertex
fn vertexMain(@builtin(vertex_index) i : u32,
            @builtin(instance_index) diskNum : u32,
            @location(0) position: vec2f) -> VertexOutput {
    let offset = diskOffsets[diskNum];
    let color = colors[diskNum];
    var output : VertexOutput;
    // output.position = vec4f(position + offset, 0, 1);
    output.position = vec4f(position, 0, 1);
    output.color = color;
    output.texelCoords = position * 0.5 + vec2f(0.5);
    return output;
}

@fragment
fn fragmentMain(in: VertexOutput) -> @location(0) vec4f{
    let texelCoordinates = vec2i(in.texelCoords * vec2f(textureDimensions(input_texture)));
    let texcolor: vec3f = textureLoad(input_texture, texelCoordinates, 0).xyz + vec3f(0.1);
    // texcolor += vec3f(0.1, 0.1, 0.1);
    // texcolor.x = 1.0;
    return vec4f(texcolor, 1.0);
    // return vec4f(1.0);
}