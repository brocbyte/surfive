@group(0) @binding(0) var<storage, read_write > diskOffsets : array<f32>;
@group(0) @binding(1) var<storage, read_write > diskVelocities : array<f32>;
@group(0) @binding(2) var<storage>params : array<f32, 3>;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) global_id: vec3u) {
    let index = global_id.x;
    let diskCount = u32(params[0]);
    if (index >= 2 * diskCount) {
        return;
    }
    let radius = params[1];
    let dt = 0.1 * params[2];
    var position = diskOffsets[index];
    position += diskVelocities[index] * dt;
    if (position > 1 - radius) {
        diskVelocities[index] = -diskVelocities[index];
        position = 2 * (1 - radius) - position;
    } else if (position < -1 + radius) {
        diskVelocities[index] = -diskVelocities[index];
        position = 2 * (-1 + radius) - position;
    }
    diskOffsets[index] = position;
}