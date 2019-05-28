



#ifndef FIXED_PRECISION_H_
#define FIXED_PRECISION_H_

// TODO: implement mixed precision

template<typename REP, int PREC, typename MULREP=int64_t>
class fixed_point
{
private:
    REP value;
    inline explicit fixed_point(REP val)
    {
        value = val;
    }

public:
    inline fixed_point()
    {
        value = 0;
    }

    inline explicit fixed_point(int int_value, int orig_prec)
    {
        value = int_value << (PREC - orig_prec);
    }

    inline explicit fixed_point(float float_value)
    {
        value = float_value * (float)(1 << PREC);
    }

    inline explicit fixed_point(double float_value)
    {
        value = float_value * (double)(1 << PREC);
    }

    explicit operator int () const
    {
        return value >> PREC;
    }

    explicit operator float () const
    {
        return (float)value / (float)(1 << PREC);
    }

    explicit operator double () const
    {
        return (double)value / (double)(1 << PREC);
    }

    fixed_point<REP, PREC, MULREP> operator+(const fixed_point<REP, PREC, MULREP>& fp) const
    {
        return fixed_point<REP, PREC, MULREP>(value + fp.value);
    }

    fixed_point<REP, PREC, MULREP> operator-(const fixed_point<REP, PREC, MULREP>& fp) const
    {
        return fixed_point<REP, PREC, MULREP>(value - fp.value);
    }

    fixed_point<REP, PREC, MULREP> operator*(const fixed_point<REP, PREC, MULREP>& fp) const
    {
        return fixed_point<REP, PREC, MULREP>((REP)(((MULREP)value * (MULREP)fp.value) >> PREC));
    }

    fixed_point<REP, PREC, MULREP> operator/(const fixed_point<REP, PREC, MULREP>& fp) const
    {
        return fixed_point<REP, PREC, MULREP>((REP)((((MULREP)value) << PREC) / (MULREP)fp.value));
    }
};




#endif