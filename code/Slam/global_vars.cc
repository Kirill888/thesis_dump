#include "timeLog.h"

TLOG_DECLARE;

#undef ZEROS_3x3
#undef ZEROS_2x2
#undef I_3x3
#undef I_2x2

double ZEROS_3x3[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
double ZEROS_2x2[2][2] = {{0,0},{0,0}};

double I_3x3[3][3] = {{1,0,0},
                            {0,1,0},
                            {0,0,1}};

double I_2x2[2][2] = {{1,0},
                            {0,1}};

class MappingProcessInterface;

MappingProcessInterface *mappingProcess = NULL;
