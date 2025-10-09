@group(0) @binding(0) var output_texture: texture_storage_2d<rgba8unorm, write>;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) global_id: vec3u) {
    let index = global_id.x;
    // 256 x 256 texture
    let resolution: vec2u = vec2u(256u, 256u);
    let fragCoord : vec2u = vec2u(index % resolution.x, index / resolution.y);
    var uv : vec2f = vec2f(f32(fragCoord.x), f32(fragCoord.y));
    uv = uv / f32(resolution.x); // 0..1
    uv = uv - vec2f(0.5);

    textureStore(output_texture, fragCoord, vec4f(length(uv), 0.0, 0.0, 1.0));
}