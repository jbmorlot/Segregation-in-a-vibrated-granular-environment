
#ifndef INTEGRATION_RK4_H
#define INTEGRATION_RK4_H

#include "integration.h"
#include "world.h"

void IntegrationRK4_func(double Step, struct TWorld* World);
TIntegrationMethod IntegrationRK4(void);

#endif // INTEGRATION_RK4_H
