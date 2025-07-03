#ifndef FILTERS_H
#define FILTERS_H

#include <vector>
#include <deque>
#include <string>
#include <algorithm>


class FIRFilter {
private:
    std::vector<float> coefficients;
    std::deque<float> buffer;

public:
    /**
     * Constructor for FIRFilter
     * @param coeffs Coefficients of the FIR filter
     */
    FIRFilter(const std::vector<float>& coeffs)
        : coefficients(coeffs), buffer(coeffs.size(), 0.0) {}

    float filter(float x) {
        buffer.pop_back();               
        buffer.push_front(x);        

        float output = 0.0;
        for (size_t i = 0; i < coefficients.size(); ++i) {
            output += coefficients[i] * buffer[i];
        }
        return output;
    }

    void reset() {
        std::fill(buffer.begin(), buffer.end(), 0.0);
    }

};

#endif // FILTERS_H