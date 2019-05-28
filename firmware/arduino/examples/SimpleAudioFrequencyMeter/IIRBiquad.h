



#ifndef IIR_BIQUAD_H_
#define IIR_BIQUAD_H_

// TODO: implement mixed precision
template<typename TYPE>
class iir_biquad
{
  const TYPE a[2], b[3];
  TYPE w[3];

public:
  iir_biquad(TYPE a1, TYPE a2, TYPE b0, TYPE b1, TYPE b2) : 
    a({a1, a2}),
    b({b0, b1, b2})
  {}

  void put(const TYPE& value)
  {        
    w[2] = w[1];
    w[1] = w[0];
    w[0] = value + a[0] * w[1] + a[1] * w[2];
  }

  TYPE get()
  {
    return b[0] * w[0] + b[1] * w[1] + b[2] * w[2];
  }
};


#endif