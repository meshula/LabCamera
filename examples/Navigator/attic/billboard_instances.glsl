//------------------------------------------------------------------------------
//  shaders for instancing-sapp sample
//------------------------------------------------------------------------------
@ctype mat4 hmm_mat4

@vs billboard_instances_vs
uniform billboard_instances_vs_params {
    mat4 v_t;
    mat4 mvp;
};

in vec3 pos;
in vec4 color0;
in vec3 inst_pos;

out vec4 color;
out vec2 uv;

void main() {
    vec3 camx = v_t[0].xyz;//vec3(v_t[0][0], v_t[1][0], v_t[2][0]);
    vec3 camy = v_t[1].xyz;//vec3(v_t[0][1], v_t[1][1], v_t[2][1]);
    vec4 new_pos = vec4(pos.xxx * camx + pos.yyy * camy + inst_pos.zzz * v_t[2].xyz + inst_pos, 1.0);
    gl_Position = mvp * new_pos;
    color = color0;
    uv = pos.xy * 10.0; // @TODO, currently quad is 0.1 because size is not yet an input
}
@end

@fs billboard_instances_fs
in vec4 color;
in vec2 uv;
out vec4 frag_color;
void main() {
    frag_color = vec4(uv.x * 0.5 + 0.5, uv.y * 0.5 + 0.5, 0.0, 1.0);
}
@end

@program billboard_instances billboard_instances_vs billboard_instances_fs
