// LowPassFilter.h
#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

class LowPassFilter {
public:
    LowPassFilter(float alpha);
    void update(float rawX);
    float getFiltered() const;

private:
    float alpha;
    float lastFilteredX;
};

#endif // LOW_PASS_FILTER_H
