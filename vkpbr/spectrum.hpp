#ifndef PBRT_LEARN_CORE_SPECTRUM_H_
#define PBRT_LEARN_CORE_SPECTRUM_H_

#include <cassert>
#include <cmath>

template <int nSpectrumSamples>
class CoefficientSpectrum
{
  public:
    // Default constructor: initializes a spectrum with constant value
    //   for all wavelengths
    explicit CoefficientSpectrum(float v = 0.0f)
    {
        for (int i = 0; i < kNSamples; ++i) {
            coeff_[i] = v;
        }
    }
    // +, +=, -, -=, *, *=, /, /=, unary -, ==, !=
    // TODO: Overloaded arithmetic operations.
    // ---------------------------------------
    CoefficientSpectrum& operator+=(const CoefficientSpectrum& rhs)
    {
        for (int i = 0; i < kNSamples; ++i) {
            coeff_[i] += rhs.coeff_[i];
        }
        return *this;
    }
    CoefficientSpectrum operator+(const CoefficientSpectrum& rhs) const
    {
        CoefficientSpectrum res = *this;
        return (res) += rhs;
    }
    CoefficientSpectrum& operator*=(const CoefficientSpectrum& rhs)
    {
        for (int i = 0; i < kNSamples; ++i) {
            coeff_[i] *= rhs.coeff_[i];
        }
        return *this;
    }
    CoefficientSpectrum operator*(const CoefficientSpectrum& rhs) const
    {
        CoefficientSpectrum res = *this;
        res *= rhs;
        return res;
    }
    CoefficientSpectrum& operator*=(float s)
    {
        for (int i = 0; i < kNSamples; ++i) {
            coeff_[i] *= s;
        }
        return *this;
    }

    CoefficientSpectrum operator*(float s) const
    {
        CoefficientSpectrum res = *this;
        res *= s;
        return res;
    }
    CoefficientSpectrum& operator/=(float s)
    {
        for (int i = 0; i < kNSamples; ++i) {
            coeff_[i] /= s;
        }
        return *this;
    }
    CoefficientSpectrum operator/(float s) const
    {
        CoefficientSpectrum res = *this;
        res /= s;
        return res;
    }

    CoefficientSpectrum& operator-=(const CoefficientSpectrum& rhs)
    {
        for (int i = 0; i < kNSamples; ++i) {
            coeff_[i] -= rhs.coeff_[i];
        }
        return *this;
    }
    CoefficientSpectrum operator-(const CoefficientSpectrum& rhs) const
    {
        CoefficientSpectrum res = *this;
        res -= rhs;
        return res;
    }


    // Returns true if all coefficients are 0.
    bool IsBlack() const
    {
        for (int i = 0; i < kNSamples; ++i) {
            if (coeff_[i] != 0)
                return false;
        }
        return true;
    }

    // Returns true if one of the coefficients is NaN.
    bool HasNaNs() const
    {
        for (int i = 0; i < kNSamples; i++) {
            if (std::isnan(coeff_[i]))
                return true;
        }
        return false;
    }

    // Returns true if one of the coefficients is negative.
    bool HasNegs() const
    {
        for (int i = 0; i < kNSamples; ++i) {
            if (coeff_[i] < 0)
                return true;
        }
        return false;
    }

    float& operator[](int i)
    {
        assert(i >= 0 && i < kNSamples);
        return coeff_[i];
    }
    float operator[](int i) const
    {
        assert(i >= 0 && i < kNSamples);
        return coeff_[i];
    }

    friend CoefficientSpectrum sqrt(const CoefficientSpectrum& s)
    {
        CoefficientSpectrum res;
        for (int i = 0; i < kNSamples; ++i) {
            res.coeff_[i] = std::sqrt(s.coeff_[i]);
        }
    }

    CoefficientSpectrum Clamp(float low = 0.f, float high = INFINITY) const
    {
        CoefficientSpectrum res;
        for (int i = 0; i < kNSamples; ++i) {
            res.coeff_[i] = coeff_[i] < low ? low : (coeff_[i] > high ? high : coeff_[i]);
        }
        return res;
    }

    // data
    static const int kNSamples = nSpectrumSamples;

  protected:
    // private data
    float coeff_[nSpectrumSamples] = {0};
};

template <int N>
CoefficientSpectrum<N> operator*(float scale, const CoefficientSpectrum<N>& s)
{
    return s * scale;
}

// +------------------------------------------------------------------+
// | SampledSpectrum                                                  |
// +------------------------------------------------------------------+

static constexpr int kSampledLambdaStart = 400;
static constexpr int kSampledLambdaEnd   = 700;
static constexpr int kNSpectralSamples   = 60;
static constexpr int kSampleWidth = (kSampledLambdaEnd - kSampledLambdaStart) / kNSpectralSamples;
static_assert(kSampleWidth * kNSpectralSamples == kSampledLambdaEnd - kSampledLambdaStart,
              "Number of sample not divisible by span.");

// Represents an SPD with uniformly spaced samples 400:5:700.
class SampledSpectrum : public CoefficientSpectrum<kNSpectralSamples>
{
  public:
    // methods

    // Default constructor and conversion constructor from base class.
    explicit SampledSpectrum(float v = 0.f)
        : CoefficientSpectrum(v)
    {}
    SampledSpectrum(const CoefficientSpectrum<kNSamples>& cs)
        : CoefficientSpectrum{cs}
    {}

    // Takes array of SPD sample values `v` at given wavelengths `lambda`,
    //   and define a piecewise linear function to represent the SPD.
    static SampledSpectrum FromSampled(const float* lambdas, const float* vs, int n);

    // Computes the approximated Riemann sum of
    //   integrate(l, S(l)*X(l)) / integrate(l, X(l))
    // as
    //   sum(i:nSamples, S(i)*X(i)) / sum(i:nSamples, 1)
    void ToXyz(float xyz[3]) const
    {
        xyz[0] = xyz[1] = xyz[2] = 0.0f;
        for (int i = 0; i < kNSamples; ++i) {
            xyz[0] += X.coeff_[i] * coeff_[i];
            xyz[1] += Y.coeff_[i] * coeff_[i];
            xyz[2] += Z.coeff_[i] * coeff_[i];
        }
        xyz[0] *= kSampleWidth / kCIEYIntegral;
        xyz[1] *= kSampleWidth / kCIEYIntegral;
        xyz[2] *= kSampleWidth / kCIEYIntegral;
    }

    // Computes the rgb representation of the spectrum.
    void ToRgb(float rgb[3]) const
    {
        float xyz[3];
        ToXyz(xyz);
        XyzToRgb(xyz, rgb);
    }

    // Computes luminance by evaluating the y component.
    float Luminance() const
    {
        float res = 0.0f;
        for (int i = 0; i < kNSamples; ++i) {
            res += Y.coeff_[i] * coeff_[i];
        }
        res *= kSampleWidth / kCIEYIntegral;
        return res;
    }

    static void XyzToRgb(const float xyz[3], float rgb[3])
    {
        rgb[0] = 3.240479f * xyz[0] - 1.537150f * xyz[1] - 0.498535f * xyz[2];
        rgb[1] = -0.969256f * xyz[0] + 1.875991f * xyz[1] + 0.041556f * xyz[2];
        rgb[2] = 0.055648f * xyz[0] - 0.204043f * xyz[1] + 1.057311f * xyz[2];
    }

    static void RgbToXyz(const float rgb[3], float xyz[3])
    {
        xyz[0] = 0.412453f * rgb[0] + 0.357580f * rgb[1] + 0.180423f * rgb[2];
        xyz[1] = 0.212671f * rgb[0] + 0.715160f * rgb[1] + 0.072169f * rgb[2];
        xyz[2] = 0.019334f * rgb[0] + 0.119193f * rgb[1] + 0.950227f * rgb[2];
    }

    // initializes SampledSpectrum::(X, Y, Z).
    static void Init();

  private:
    static SampledSpectrum X, Y, Z;
    static constexpr float kCIEYIntegral = 106.856895f;
};

// XYZ color space
// Tristimulus theory: all visible SPDs can be accurately represented
//   for human observers with x, y, z.
// x_l = integrate(l, S(l)*X(l)) / integrate(l, Y(l))
// y_l = integrate(l, S(l)*Y(l)) / integrate(l, Y(l))
// z_l = integrate(l, S(l)*Z(l)) / integrate(l, Y(l))
// with X(.), Y(.), Z(.) defined as follows:

constexpr int      kNumCIESamples = 471;   // from 360nm to 830nm
extern const float CIE_X[kNumCIESamples];  // coefficients
extern const float CIE_Y[kNumCIESamples];
extern const float CIE_Z[kNumCIESamples];
extern const float CIE_lambda[kNumCIESamples];  // wavelengths

inline void XyzToRgb(const float xyz[3], float rgb[3])
{
    rgb[0] = 3.240479f * xyz[0] - 1.537150f * xyz[1] - 0.498535f * xyz[2];
    rgb[1] = -0.969256f * xyz[0] + 1.875991f * xyz[1] + 0.041556f * xyz[2];
    rgb[2] = 0.055648f * xyz[0] - 0.204043f * xyz[1] + 1.057311f * xyz[2];
}

inline void RgbToXyz(const float rgb[3], float xyz[3])
{
    xyz[0] = 0.412453f * rgb[0] + 0.357580f * rgb[1] + 0.180423f * rgb[2];
    xyz[1] = 0.212671f * rgb[0] + 0.715160f * rgb[1] + 0.072169f * rgb[2];
    xyz[2] = 0.019334f * rgb[0] + 0.119193f * rgb[1] + 0.950227f * rgb[2];
}

class RGBSpectrum : public CoefficientSpectrum<3>
{
  public:
    explicit RGBSpectrum(float v = 0.0f)
        : CoefficientSpectrum<3>(v)
    {}
    RGBSpectrum(const CoefficientSpectrum<3>& cs)
        : CoefficientSpectrum<3>(cs)
    {}

    static RGBSpectrum FromRgb(const float rgb[3])
    {
        RGBSpectrum s;
        s.coeff_[0] = rgb[0];
        s.coeff_[1] = rgb[1];
        s.coeff_[2] = rgb[2];
        return s;
    }
    void ToRgb(float* rgb) const
    {
        rgb[0] = coeff_[0];
        rgb[1] = coeff_[1];
        rgb[2] = coeff_[2];
    }
    void               ToXyz(float xyz[3]) const { RgbToXyz(coeff_, xyz); }
    static RGBSpectrum FromXyz(const float xyz[3])
    {
        RGBSpectrum r;
        XyzToRgb(xyz, r.coeff_);
        return r;
    }

    float y() const
    {
        const float kYWeight[3] = {0.212671f, 0.715160f, 0.072169f};
        return kYWeight[0] * coeff_[0] + kYWeight[1] * coeff_[1] + kYWeight[2] * coeff_[2];
    }
};

using Spectrum = RGBSpectrum;

// clamp(t, s0, s1);

// Computes the emit radiance of a blackbody given temperature and wavelengths.
void Blackbody(const float* lambdas, int n, float temperature, float* emit_radiance);

void BlackbodyNormalized(const float* lambdas, int n, float temperature, float* emit_radiance);

#endif  // PBRT_LEARN_CORE_SPECTRUM_H_
