#include <iostream>
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include "my_types.h"

namespace g2o {

    G2O_REGISTER_TYPE(BIAS_VERTEX, BiasVertex);
    G2O_REGISTER_TYPE(GPS_EDGE_PRIOR, GPSEdgePrior);
    G2O_REGISTER_TYPE(DOPPLER_EDGE, DopplerEdge);


}