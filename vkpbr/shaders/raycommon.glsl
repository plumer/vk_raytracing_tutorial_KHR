struct hitPayload
{
  vec3 hitValue;
  // RNG seed. It is meant to be changed every time a random number is generated using it.
  uint seed;
  // Length of the tracing path. If it exceeds some threshold, path tracing should stop.
  uint depth;

  vec3 ray_origin;
  vec3 ray_direction;
  vec3 weight;
};
