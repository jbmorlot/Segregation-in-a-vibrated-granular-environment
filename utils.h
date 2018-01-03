

#ifndef UTILS_H
#define UTILS_H

#define EPSILON 0.00001
#define MIN(a,b) (a < b ? a : b)
#define MAX(a,b) (a > b ? a : b)

int solve_second_order(double a, double b, double c, double* root1, double* root2);

#endif // UTILS_H

