@ctype mat4 gizmo_mat4

@vs vs
uniform vs_params {
    mat4 mvp;
};

in vec3 position;
in vec3 normal0;
in vec4 color0;

out vec3 normal;
out vec4 color;

void main() {
    gl_Position = mvp * vec4(position, 1);
    color = color0;
    normal = normal0;
}
@end

@fs fs
in vec3 normal;
in vec4 color;
out vec4 frag_color;

void main() {
    frag_color = color;
}
@end

@program gizmo vs fs
