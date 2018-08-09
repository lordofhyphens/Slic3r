#ifndef slic3r_ArcFitting_hpp_
#define slic3r_ArcFitting_hpp_

#include "libslic3r.h"
#include "GCode.hpp"
#include "GCodeReader.hpp"

namespace Slic3r {

enum ArcType {
    CW,CCW,LINE
};

class ArcPath : public Polyline {
    public:
    Points centers;
    std::vector<ArcType> arc_types;
};

std::vector<size_t> arc_detection(Polyline &pl, size_t N, double epsilon);
std::vector<std::pair<size_t,size_t>> arc_fitting(std::vector<size_t> &arcs, Polyline &pl);
ArcPath build_arcpath(std::vector<std::pair<size_t,size_t>> &arcs, Polyline &pl);

ArcPath arcify(Polyline &pl,size_t N, double e);
class ArcFitting {
    public:
    bool enable;
    
    ArcFitting(const PrintConfig &config)
        : enable(false), _config(&config)
    {
        this->_reader.apply_config(*this->_config);
    };
    std::string process_layer(const std::string &gcode);
    
    private:
    const PrintConfig* _config;
    GCodeReader _reader;
};

}

#endif
