#include <log4cplus/initializer.h>

#include "imalgs/AircraftState.h"

#if __cplusplus != 201703L
#error "The installed package must support a C++17 consumer"
#endif

int main() {
   log4cplus::Initializer initializer;
   interval_management::open_source::AircraftState state;
   return Units::FeetPerSecondSpeed(state.GetGroundSpeed()).value() == 0.0 ? 0 : 1;
}
