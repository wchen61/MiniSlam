//
//  Time.h
//  MiniSlam
//
//  Created by  weiwu.cww on 2017/11/7.
//  Copyright © 2017年  weiwu.cww. All rights reserved.
//

#ifndef Time_h
#define Time_h

#include <iostream>
#include <chrono>

namespace MiniSlam {

class Timer {
public:
    Timer() : beg_(clock_::now()) {}
    void reset() {beg_ = clock_::now();}
    double elapsed() const {
        double time_span = std::chrono::duration_cast<second_> (clock_::now() - beg_).count();
        return time_span;
    }
    
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1>> second_;
    std::chrono::time_point<clock_> beg_;
};

}
#endif /* Time_h */
