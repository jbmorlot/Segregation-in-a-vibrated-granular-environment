
#ifndef INTEGRATION_RK2_H
#define INTEGRATION_RK2_H

#include "integration.h"
#include "world.h"

void IntegrationRK2_func(double Step, struct TWorld* World);
TIntegrationMethod IntegrationRK2(void);

#endif // INTEGRATION_RK2_H
