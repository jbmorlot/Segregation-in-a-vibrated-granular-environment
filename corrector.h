
#ifndef CORRECTOR_H
#define CORRECTOR_H

#include "pair.h"

// Le correcteur est appelé pour chaque paire afin d'éventuellement corriger leur position
typedef void (*TCorrector)(double TimeStep, TPair* Pair);

#endif // CORRECTOR_H
