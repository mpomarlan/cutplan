#ifndef __CUTPLAN_EXAMPLE_MANEUVERS_H__

#define __CUTPLAN_EXAMPLE_MANEUVERS_H__

#include <cutplan/Maneuver.h>

namespace cutplan
{

class Brush: public Maneuver
{
public:
protected:
    virtual bool shapeFromSample(std::vector<tPointSpec> const& samples);
    virtual bool shapeFromSample(std::vector<tPointSpec> const& samples, tPointSpec const& direction);
};
class Burr: public Maneuver
{
public:
protected:
    virtual bool shapeFromSample(std::vector<tPointSpec> const& samples);
    virtual bool shapeFromSample(std::vector<tPointSpec> const& samples, tPointSpec const& direction);
};
class Wiper: public Maneuver
{
public:
protected:
    virtual bool shapeFromSample(std::vector<tPointSpec> const& samples);
    virtual bool shapeFromSample(std::vector<tPointSpec> const& samples, tPointSpec const& direction);
};

}
#endif
