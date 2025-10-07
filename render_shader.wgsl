struct VertexOutput {
    @builtin(position) position : vec4f,
    @location(0) @interpolate(flat) color : vec4f
}

@group(0) @binding(0) var<storage> diskOffsets : array<vec2f>;
@group(0) @binding(1) var<storage> colors : array<vec4f>;
@vertex
fn vertexMain(@builtin(vertex_index) i : u32,
            @builtin(instance_index) diskNum : u32,
            @location(0) position: vec2f) -> VertexOutput {
    let offset = diskOffsets[diskNum];
    let color = colors[diskNum];
    var output : VertexOutput;
    output.position = vec4f(position + offset, 0, 1);
    output.color = color;
    return output;
}

@fragment
fn fragmentMain( @location(0) @interpolate(flat) color : vec4f) -> @location(0) vec4f{
    return color;
}