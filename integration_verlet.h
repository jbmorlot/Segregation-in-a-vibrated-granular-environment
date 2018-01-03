
#ifndef __INTEGRATION_VERLET_H
#define __INTEGRATION_VERLET_H

#include "integration.h"
#include "world.h"

void IntegrationVerlet_func(double Step, struct TWorld* World);
TIntegrationMethod IntegrationVerlet(void);

#endif // __INTEGRATION_VERLET_H
