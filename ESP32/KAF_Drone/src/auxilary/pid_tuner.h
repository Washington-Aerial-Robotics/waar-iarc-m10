#include "../core/kaf_drone.h"

void pindtuner_simulationState( double matrix[13] );

peripheral pidtuner_reset();
void pidtuner_step( void* imuData );