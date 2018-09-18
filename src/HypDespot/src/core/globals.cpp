#include <despot/core/globals.h>

using namespace std;

namespace despot {
namespace Globals {

Config config;
ExecTracker tracker;
const double INF = 1e8;
const double TINY = 1e-8;
const double POS_INFTY = std::numeric_limits<double>::is_iec559 ?
	numeric_limits<double>::infinity() :
	numeric_limits<double>::max();
const double NEG_INFTY = -POS_INFTY;
const uint64_t RESIDUAL_OBS = numeric_limits<uint64_t>::max();
} // namespace Globals
} // namespace despot
