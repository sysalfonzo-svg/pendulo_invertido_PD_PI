#pragma once
// Generado por matlab/disenar_control.m. Ruedas en orden fisico I/D.
namespace gan {
constexpr float T = 5.000000000e-03f;
constexpr float kp = 3.203358361e+01f;
constexpr float kd = 6.055437028e+00f;
constexpr float kx = 2.199705976e+00f;
constexpr float kv = 3.785922526e+00f;
constexpr float vMaxRef = 2.500000000e-01f;
constexpr float pwmMax = 7.000000000e+01f;
constexpr float filtro = 1.200000000e-02f;
constexpr float suavizado = 2.000000000e+00f;
constexpr float K[2] = {4.545230071e-03f, 3.764832276e-03f};
constexpr float tau[2] = {1.688000000e-01f, 1.573000000e-01f};
constexpr float kpw[2] = {6.752333459e+02f, 7.596619956e+02f};
constexpr float kiw[2] = {4.000197547e+03f, 4.829383316e+03f};
constexpr float muerto[2] = {2.500000000e+01f, 3.000000000e+01f};
}
