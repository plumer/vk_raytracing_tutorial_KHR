struct hitPayload
{
  vec3 hitValue;
  // RNG seed. It is meant to be changed every time a random number is generated using it.
  uint seed;
  // Length of the tracing path. If it exceeds some threshold, path tracing should stop.
  uint depth;

  // The rendering equation goes
  //              /
  // L (w ) = L + |     L (w ) f(w , w ) |cos \theta | dw
  //  o  o     e / H(n)  i  i     o   i             i    i
  //
  // where hitValue = L_e and weight = f(w_o, w_i) |cos \theta_i| / p(w_i).
  // p(w_i) is the probability distribution function of the sample generator.

  vec3 ray_origin;
  vec3 ray_direction;
  vec3 weight;
};
